// @file QSM.swift
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

/// One cylinder in a quantitative structure model.
///
/// Column names match the R `qsm` data frame: `startX` through `radius`, plus
/// `quality` from the CSV writer.
public struct Cylinder: Sendable {
    public var startX: Double
    public var startY: Double
    public var startZ: Double
    public var endX: Double
    public var endY: Double
    public var endZ: Double
    public var cylID: Int32
    public var parentID: Int32
    public var axisID: Int32
    public var branchOrder: Int32
    public var distToRoot: Double
    public var subtreeLength: Double
    public var radius: Double
    public var quality: Int32
    /// `pi * radius^2 * length`, filled by ``qsmVolume(_:)``.
    public var volume: Double

    public init(
        startX: Double,
        startY: Double,
        startZ: Double,
        endX: Double,
        endY: Double,
        endZ: Double,
        cylID: Int32,
        parentID: Int32,
        axisID: Int32,
        branchOrder: Int32,
        distToRoot: Double,
        subtreeLength: Double,
        radius: Double,
        quality: Int32 = 0,
        volume: Double = 0
    ) {
        self.startX = startX
        self.startY = startY
        self.startZ = startZ
        self.endX = endX
        self.endY = endY
        self.endZ = endZ
        self.cylID = cylID
        self.parentID = parentID
        self.axisID = axisID
        self.branchOrder = branchOrder
        self.distToRoot = distToRoot
        self.subtreeLength = subtreeLength
        self.radius = radius
        self.quality = quality
        self.volume = volume
    }

    var length: Double {
        let dx = endX - startX
        let dy = endY - startY
        let dz = endZ - startZ
        return (dx * dx + dy * dy + dz * dz).squareRoot()
    }
}

/// A quantitative structure model: cylinder rows plus the R attributes `id`, `name`, `message`, and `crs`.
public final class QSM {
    public var cylinders: [Cylinder]
    /// R attribute `id`.
    public var id: Int?
    /// R attribute `name`.
    public var name: String?
    /// R attribute `message`. One entry per log line.
    public var message: [String]
    /// Coordinate reference system as WKT, or `""` when unset. Swift form of `st_crs`.
    public var crs: String

    public init(
        cylinders: [Cylinder] = [],
        id: Int? = nil,
        name: String? = nil,
        message: [String] = [],
        crs: String = ""
    ) {
        self.cylinders = cylinders
        self.id = id
        self.name = name
        self.message = message
        self.crs = crs
    }
}

/// Build a QSM for one tree.
///
/// Wood points are the ones used. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter tree: A single-tree cloud.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: The cylinder model. `crs` is copied from the cloud.
///
/// ## See Also
/// ``qsmWrite(_:to:binary:)``
/// ``qsmRead(_:)``
/// ``qsmDbh(_:breastHeight:)``
/// ``qsmStats(_:breastHeight:display:)``
public func qsm(_ tree: PointCloud, params: ArborParameters = arborParametersDefault) throws -> QSM {
    try tree.validate()
    let model = try Engine.qsm(tree, params)
    if model.crs.isEmpty {
        model.crs = tree.crs
    }
    return model
}
