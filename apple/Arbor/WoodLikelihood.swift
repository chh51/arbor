// @file WoodLikelihood.swift
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

/// Local wood likelihood.
///
/// Estimates the local anisotropy of each point from its k nearest neighbours.
/// The neighbour count is `params.woodLikelihood.k`. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter cloud: The cloud to score.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: The same cloud, with `pwood` written on every point.
public func woodLikelihood(_ cloud: PointCloud, params: ArborParameters = arborParametersDefault) throws -> PointCloud {
    try cloud.validate()
    return try Engine.anisotropy(cloud, k: params.woodLikelihood.k)
}
