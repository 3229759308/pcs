#pragma once

#include "core/handle/points.hh"
#include "core/units/points.hh"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtk/vtkMatrix4x4.h>

#include <algorithm>
#include <cstdint>

using namespace pcs;

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

constexpr auto error_code = -1;

struct PointsHandle::Impl final {

    std::unique_ptr<PointsUnit> unit;
    std::shared_ptr<PointCloud> points;
    std::optional<double> projection_plane_z;
    vtkSmartPointer<vtkProperty> original_property;

    auto apply_projection() noexcept -> void {
        if (unit == nullptr) {
            return;
        }
        auto actor = unit->actor();
        if (projection_plane_z.has_value()) {
            if (original_property == nullptr) {
                original_property = vtkSmartPointer<vtkProperty>::New();
                original_property->DeepCopy(actor->GetProperty());
            }
            auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
            matrix->Identity();
            matrix->SetElement(2, 2, 0.0);
            // Keep the overlay just above the opaque PNG to avoid depth fighting.
            matrix->SetElement(2, 3, *projection_plane_z);
            actor->SetUserMatrix(matrix);
            actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
            actor->GetProperty()->SetOpacity(0.5);
            actor->GetProperty()->LightingOff();
        } else {
            actor->SetUserMatrix(nullptr);
            if (original_property != nullptr) {
                actor->GetProperty()->DeepCopy(original_property);
                original_property = nullptr;
            }
        }
        unit->set_coordinate_visibility(visible && coordinate_visible
            && !projection_plane_z.has_value());
    }

    bool coordinate_visible = true;
    bool visible            = true;
    std::vector<Renderer*> attached_renderers;

    auto rebind_unit(std::unique_ptr<PointsUnit> next_unit) noexcept -> void {
        if (unit != nullptr) {
            for (auto* renderer : attached_renderers) {
                if (renderer != nullptr) {
                    renderer->detach(*unit);
                }
            }
        }

        original_property = nullptr;
        unit              = std::move(next_unit);
        if (unit == nullptr) {
            return;
        }

        unit->set_visibility(visible);
        apply_projection();

        for (auto* renderer : attached_renderers) {
            if (renderer != nullptr) {
                renderer->attach(*unit);
            }
        }
    }

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        points = std::make_shared<PointCloud>();
        if (error_code == pcl::io::loadPCDFile(path, *points)) {
            return std::unexpected { "Failed to read pointcloud from filesystem" };
        }

        auto next_unit = std::make_unique<PointsUnit>(points->points);
        rebind_unit(std::move(next_unit));
        return { };
    }

    auto load_from_positions(std::vector<PointsHandle::Position> const& positions) noexcept
        -> std::expected<void, std::string_view> {
        points = std::make_shared<PointCloud>();
        points->points.reserve(positions.size());

        for (const auto& [x, y, z] : positions) {
            points->points.emplace_back(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }

        points->width  = static_cast<std::uint32_t>(points->points.size());
        points->height = 1;

        auto next_unit = std::make_unique<PointsUnit>(points->points);
        rebind_unit(std::move(next_unit));
        return { };
    }

    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        if (points == nullptr) {
            return std::unexpected { "Points is not loaded" };
        }
        if (error_code == pcl::io::savePCDFile(path, *points, false)) {
            return std::unexpected { "Failed to save pointcloud to filesystem" };
        }
        return { };
    }

    auto get_positions() const noexcept -> std::vector<PointsHandle::Position> {
        auto result = std::vector<PointsHandle::Position> { };
        if (points == nullptr) {
            return result;
        }

        result.reserve(points->points.size());
        for (const auto& point : points->points) {
            result.emplace_back(point.x, point.y, point.z);
        }

        return result;
    }

    auto set_coordinate_visibility(bool on) noexcept -> void {
        coordinate_visible = on;
        if (unit != nullptr) {
            unit->set_coordinate_visibility(visible && on && !projection_plane_z.has_value());
        }
    }

    auto set_visibility(bool on) noexcept -> void {
        visible = on;
        if (unit != nullptr) {
            unit->set_visibility(on);
            unit->set_coordinate_visibility(
                on && coordinate_visible && !projection_plane_z.has_value());
        }
    }

    auto attach_renderer(Renderer& renderer) noexcept -> void {
        if (std::find(attached_renderers.begin(), attached_renderers.end(), &renderer)
            == attached_renderers.end()) {
            attached_renderers.push_back(&renderer);
        }

        if (unit != nullptr) {
            renderer.attach(*unit);
        }
    }

    auto detach_renderer(Renderer& renderer) noexcept -> void {
        if (unit != nullptr) {
            renderer.detach(*unit);
        }

        std::erase(attached_renderers, &renderer);
    }

    auto coordinate_visibility() const noexcept -> bool { return coordinate_visible; }
};
