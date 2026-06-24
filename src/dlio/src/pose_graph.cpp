#include "pose_graph.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <cmath>
#include <memory>
#include <utility>

namespace small_dlio {

    namespace {

        bool isFiniteIsometry(
            const Eigen::Isometry3d &pose
        ) {

            return pose.matrix().allFinite();
        }

        bool isFiniteInformation(
            const Eigen::Matrix<double, 6, 6> &information
        ) {

            return information.allFinite();
        }

        // 确保信息矩阵对称
        Eigen::Matrix<double, 6, 6> sanitizedInformation(
            const Eigen::Matrix<double, 6, 6> &information
        ) {

            return 0.5 * (information + information.transpose());
        }
    }

    void PoseGraph::configure(
        const PoseGraphOptions &options
    ) {

        options_ = options;
        if (options_.max_iterations <= 0)
            options_.max_iterations = 20;
        if (!std::isfinite(options_.odom_edge_weight) ||
            options_.odom_edge_weight <= 0.0)
            options_.odom_edge_weight = 100.0;
        if (!std::isfinite(options_.loop_edge_weight) ||
            options_.loop_edge_weight <= 0.0)
            options_.loop_edge_weight = 500.0;
    }

    void PoseGraph::clear() {

        nodes_.clear();
        edges_.clear();
        node_index_.clear();
        graph_dirty_ = false;
    }

    bool PoseGraph::empty() const {

        return nodes_.empty();
    }

    size_t PoseGraph::nodeCount() const {

        return nodes_.size();
    }

    size_t PoseGraph::edgeCount() const {

        return edges_.size();
    }

    bool PoseGraph::hasNode(
        const uint32_t id
    ) const {

        return node_index_.find(id) != node_index_.end();
    }

    bool PoseGraph::addNode(
        const PoseGraphNode &node
    ) {

        if (hasNode(node.id) || !isFiniteIsometry(node.pose))
            return false;

        PoseGraphNode stored = node;
        if (options_.fix_first_node && nodes_.empty())
            stored.fixed = true;

        node_index_[stored.id] = nodes_.size();
        nodes_.push_back(stored);
        graph_dirty_ = true;
        return true;
    }

    /**
     * @brief 添加里程计边
     * @param from_id 前一帧关键帧的 id
     * @param to_id 后一帧关键帧的 id
     * @param relative_pose 相对位姿( from 到 to 坐标系的变换)
     * @param information 信息矩阵(协方差矩阵的逆，表示测量的置信度)
     * @return bool 是否成功
     *
     * edge.score 的 score 表示 GICP 配准的质量分数
     */
    bool PoseGraph::addOdomEdge(
        const uint32_t from_id,
        const uint32_t to_id,
        const Eigen::Isometry3d &relative_pose,
        const Eigen::Matrix<double, 6, 6> &information
    ) {

        PoseGraphEdge edge;
        edge.from_id = from_id;
        edge.to_id = to_id;
        edge.type = PoseGraphEdgeType::Odom;
        edge.relative_pose = relative_pose;
        edge.information = information;
        edge.score = 0.0;
        return addEdge(std::move(edge));
    }

    // 回环帧的边的添加
    bool PoseGraph::addLoopEdge(
        const uint32_t from_id,
        const uint32_t to_id,
        const Eigen::Isometry3d &relative_pose,
        const Eigen::Matrix<double, 6, 6> &information,
        const double score
    ) {

        PoseGraphEdge edge;
        edge.from_id = from_id;
        edge.to_id = to_id;
        edge.type = PoseGraphEdgeType::Loop;
        edge.relative_pose = relative_pose;
        edge.information = information;
        edge.score = score;
        return addEdge(std::move(edge));
    }

    PoseGraphOptimizationSummary PoseGraph::optimize() {

        PoseGraphOptimizationSummary summary;
        summary.node_count = static_cast<int>(nodes_.size());
        summary.edge_count = static_cast<int>(edges_.size());

        if (nodes_.empty()) {

            summary.message = "pose graph has no nodes";
            return summary;
        }

        if (edges_.empty()) {

            summary.message = "pose graph has no edges";
            return summary;
        }

        // 将稀疏矩阵分块
        using BlockSolverType =
            g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
        // 线性方程求解
        using LinearSolverType =
            g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        auto linear_solver = std::make_unique<LinearSolverType>();
        linear_solver->setBlockOrdering(false);

        auto block_solver =
            std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto algorithm =
            new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
        optimizer.setAlgorithm(algorithm);

        // 转换待优化的节点和边
        for (const auto &node : nodes_) {

            auto *vertex = new g2o::VertexSE3();
            vertex->setId(static_cast<int>(node.id));
            vertex->setEstimate(node.pose);
            vertex->setFixed(node.fixed);

            if (!optimizer.addVertex(vertex)) {

                delete vertex;
                summary.message = "failed to add g2o vertex";
                return summary;
            }
        }

        for (const auto &edge : edges_) {

            auto *from_vertex =
                optimizer.vertex(static_cast<int>(edge.from_id));
            auto *to_vertex =
                optimizer.vertex(static_cast<int>(edge.to_id));
            if (!from_vertex || !to_vertex) {

                summary.message = "edge references missing g2o vertex";
                return summary;
            }

            auto *g2o_edge = new g2o::EdgeSE3();
            g2o_edge->setVertex(0, from_vertex);
            g2o_edge->setVertex(1, to_vertex);
            g2o_edge->setMeasurement(edge.relative_pose);
            g2o_edge->setInformation(edge.information);

            if (!optimizer.addEdge(g2o_edge)) {

                delete g2o_edge;
                summary.message = "failed to add g2o edge";
                return summary;
            }
        }

        if (!optimizer.initializeOptimization()) {

            summary.message = "failed to initialize g2o optimization";
            return summary;
        }

        optimizer.computeActiveErrors();
        summary.initial_chi2 = optimizer.activeChi2();

        const int iterations = optimizer.optimize(options_.max_iterations);
        summary.iterations = iterations;

        optimizer.computeActiveErrors();
        summary.final_chi2 = optimizer.activeChi2();

        if (iterations <= 0) {

            summary.message = "g2o optimization did not run";
            return summary;
        }

        for (auto &node : nodes_) {

            const auto *vertex = dynamic_cast<const g2o::VertexSE3 *>(
                optimizer.vertex(static_cast<int>(node.id)));
            if (!vertex) {

                summary.message = "failed to read optimized g2o vertex";
                return summary;
            }

            const Eigen::Isometry3d optimized_pose = vertex->estimate();
            if (!isFiniteIsometry(optimized_pose)) {

                summary.message = "optimized pose is not finite";
                return summary;
            }

            node.pose = optimized_pose;
        }

        graph_dirty_ = false;
        summary.success = true;
        summary.message = "ok";
        return summary;
    }

    bool PoseGraph::getPose(
        const uint32_t id,
        Eigen::Isometry3d &pose
    ) const {

        const auto it = node_index_.find(id);
        if (it == node_index_.end())
            return false;

        pose = nodes_[it->second].pose;
        return true;
    }

    const std::vector<PoseGraphNode> &PoseGraph::nodes() const {

        return nodes_;
    }

    const std::vector<PoseGraphEdge> &PoseGraph::edges() const {

        return edges_;
    }

    const PoseGraphOptions &PoseGraph::options() const {

        return options_;
    }

    bool PoseGraph::addEdge(
        PoseGraphEdge edge
    ) {

        if (edge.from_id == edge.to_id)
            return false;
        if (!hasNode(edge.from_id) || !hasNode(edge.to_id))
            return false;
        if (!isFiniteIsometry(edge.relative_pose) ||
            !isFiniteInformation(edge.information))
            return false;
        if (edge.type == PoseGraphEdgeType::Loop &&
            !std::isfinite(edge.score))
            return false;

        edge.information = sanitizedInformation(edge.information);
        edges_.push_back(std::move(edge));
        graph_dirty_ = true;
        return true;
    }

} // namespace small_dlio
