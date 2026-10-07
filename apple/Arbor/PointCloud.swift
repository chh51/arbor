// @file PointCloud.swift
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

import Foundation

/// One point cloud, the Swift form of the columns Arbor reads on a lidR `LAS`.
///
/// Stages that match the R functions mutate this object and also return it.
/// A missing column is `nil`. `crs` is the Swift form of `st_crs`: a WKT string,
/// or empty when the cloud has no CRS. This type does not carry a lidR sensor flag.
public final class PointCloud {
    public var x: [Double]
    public var y: [Double]
    public var z: [Double]
    /// ASPRS classification. R name: `Classification`.
    public var classification: [Int32]?
    /// Height above ground, metres.
    public var hag: [Double]?
    /// Local wood likelihood, from ``woodLikelihood(_:params:)``.
    public var pwood: [Float]?
    /// `0` wood, `1` and `2` foliage.
    public var foliage: [Int32]?
    /// How many least-cost paths pass through the point.
    public var passage: [Int32]?
    /// `ARBORTREE`, `ARBORLOW`, `ARBORUNDERSTORY`, or `ARBORBUFFER`. R name: `UserData`.
    public var userData: [Int32]?
    public var treeID: [Int32]?
    /// Distance to the sensor, metres.
    public var range: [Double]?
    /// GPS time, seconds. R name: `gpstime`.
    public var gpsTime: [Double]?
    /// Red channel. R name: `R`.
    public var red: [UInt16]?
    /// Green channel. R name: `G`.
    public var green: [UInt16]?
    /// Blue channel. R name: `B`.
    public var blue: [UInt16]?
    public var returnNumber: [UInt8]?
    public var numberOfReturns: [UInt8]?
    /// Coordinate reference system as WKT, or `""` when unset.
    public var crs: String

    public var count: Int { x.count }

    public init(x: [Double], y: [Double], z: [Double], crs: String = "") {
        precondition(x.count == y.count && y.count == z.count, "X, Y, and Z must have the same length")
        self.x = x
        self.y = y
        self.z = z
        self.crs = crs
    }

    func validate() throws {
        func same(_ count: Int?, _ name: String) throws {
            if let count, count != self.count {
                throw ArborError.invalidArgument("\(name) has \(count) values for \(self.count) points")
            }
        }
        try same(classification?.count, "Classification")
        try same(hag?.count, "hag")
        try same(pwood?.count, "pwood")
        try same(foliage?.count, "foliage")
        try same(passage?.count, "passage")
        try same(userData?.count, "UserData")
        try same(treeID?.count, "treeID")
        try same(range?.count, "range")
        try same(gpsTime?.count, "gpstime")
        try same(red?.count, "R")
        try same(green?.count, "G")
        try same(blue?.count, "B")
        try same(returnNumber?.count, "ReturnNumber")
        try same(numberOfReturns?.count, "NumberOfReturns")
    }

    func selecting(_ included: [Bool]) -> PointCloud {
        precondition(included.count == count)
        let index = included.indices.filter { included[$0] }
        let copy = PointCloud(
            x: index.map { x[$0] },
            y: index.map { y[$0] },
            z: index.map { z[$0] },
            crs: crs
        )
        copy.classification = classification.map { column in index.map { column[$0] } }
        copy.hag = hag.map { column in index.map { column[$0] } }
        copy.pwood = pwood.map { column in index.map { column[$0] } }
        copy.foliage = foliage.map { column in index.map { column[$0] } }
        copy.passage = passage.map { column in index.map { column[$0] } }
        copy.userData = userData.map { column in index.map { column[$0] } }
        copy.treeID = treeID.map { column in index.map { column[$0] } }
        copy.range = range.map { column in index.map { column[$0] } }
        copy.gpsTime = gpsTime.map { column in index.map { column[$0] } }
        copy.red = red.map { column in index.map { column[$0] } }
        copy.green = green.map { column in index.map { column[$0] } }
        copy.blue = blue.map { column in index.map { column[$0] } }
        copy.returnNumber = returnNumber.map { column in index.map { column[$0] } }
        copy.numberOfReturns = numberOfReturns.map { column in index.map { column[$0] } }
        return copy
    }

    func emptied() -> PointCloud {
        selecting(Array(repeating: false, count: count))
    }

    func appendPoint(x: Double, y: Double, z: Double) {
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        classification?.append(Int32.min)
        hag?.append(.nan)
        pwood?.append(.nan)
        foliage?.append(Int32.min)
        passage?.append(Int32.min)
        userData?.append(Int32.min)
        treeID?.append(Int32.min)
        range?.append(.nan)
        gpsTime?.append(.nan)
        red?.append(0)
        green?.append(0)
        blue?.append(0)
        returnNumber?.append(0)
        numberOfReturns?.append(0)
    }

    func requireTreeID() throws -> [Int32] {
        guard let treeID else { throw ArborError.missingAttribute("treeID") }
        return treeID
    }

    func requireHag() throws -> [Double] {
        guard let hag else { throw ArborError.missingAttribute("hag") }
        return hag
    }
}
