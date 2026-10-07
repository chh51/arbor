// @file Homogeneization.swift
// Project: Arbor
//
// Copyright (C) 2026 Jean-Romain Roussel (r-lidar) <info @ r-lidar.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// Hybrid homogenization of a point cloud.
///
/// Keeps a barycentric voxel sample and reinjects random points. The spelling
/// matches the R function `hybrid_homogeneization`. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter cloud: The cloud to thin.
/// - Parameter res: Voxel size, metres. Default 0.02.
/// - Returns: A new cloud containing the kept points. The input is unchanged.
public func hybridHomogeneization(_ cloud: PointCloud, res: Double = 0.02) throws -> PointCloud {
    try cloud.validate()
    return try Engine.homogeneization(cloud, res: res)
}
