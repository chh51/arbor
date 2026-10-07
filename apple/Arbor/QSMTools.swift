// @file QSMTools.swift
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

/// Total cylinder volume, cubic metres.
///
/// Writes `volume` on each cylinder (`pi * radius^2 * length`) and returns the sum.
public func qsmVolume(_ model: QSM) -> Double {
    var total = 0.0
    for index in model.cylinders.indices {
        var cylinder = model.cylinders[index]
        cylinder.volume = Double.pi * cylinder.radius * cylinder.radius * cylinder.length
        model.cylinders[index] = cylinder
        total += cylinder.volume
    }
    return total
}

/// Volume of each model, in forest order.
public func qsmVolume(_ forest: QSF) -> [Double] {
    forest.models.map(qsmVolume)
}

/// Vertical extent of the cylinders, metres.
///
/// `nil` when the model has no cylinders. This is `NA` in R.
public func qsmHeight(_ model: QSM) -> Double? {
    guard !model.cylinders.isEmpty else { return nil }
    var low = model.cylinders[0].startZ
    var high = low
    for cylinder in model.cylinders {
        low = min(low, min(cylinder.startZ, cylinder.endZ))
        high = max(high, max(cylinder.startZ, cylinder.endZ))
    }
    return high - low
}

/// Height of each model. A missing height is `nil`.
public func qsmHeight(_ forest: QSF) -> [Double?] {
    forest.models.map(qsmHeight)
}

/// Log lines stored on the model.
///
/// - Parameter short: When true, each line is reduced to its bracketed label.
///   A leading `[W0]`-style code is removed with the label. See ``qsfLog(_:)``.
public func qsmMessage(_ model: QSM, short: Bool = false) -> [String] {
    guard short else { return model.message }
    return model.message.map { parseQSFMessage($0).label ?? $0 }
}

/// R attribute `id`.
public func qsmTreeID(_ model: QSM) -> Int? {
    model.id
}

/// `id` of each model, in forest order.
public func qsmTreeID(_ forest: QSF) -> [Int?] {
    forest.models.map(\.id)
}

/// Add synthetic ground points under a single tree.
///
/// Useful when a tree was isolated outside Arbor and later stages still expect
/// ground. `n` points are scattered uniformly in the XY bounding box expanded by
/// 1 m, at the cloud's minimum Z, then snapped to a 0.001 m grid. `Classification`
/// is 2 on those points when that column already exists.
///
/// - Parameter cloud: One tree.
/// - Parameter n: How many ground points to add. Default 1000.
/// - Returns: The same cloud, with the new points appended.
public func addSingleTreeGround(_ cloud: PointCloud, n: Int = 1000) throws -> PointCloud {
    try cloud.validate()
    guard cloud.count > 0 else {
        throw ArborError.invalidArgument("point cloud is empty")
    }
    guard n >= 0 else {
        throw ArborError.invalidArgument("n must be >= 0")
    }
    let z = cloud.z.min() ?? 0
    let minX = (cloud.x.min() ?? 0) - 1
    let maxX = (cloud.x.max() ?? 0) + 1
    let minY = (cloud.y.min() ?? 0) - 1
    let maxY = (cloud.y.max() ?? 0) + 1
    for _ in 0..<n {
        let x = quantize(Double.random(in: minX..<maxX), scale: 0.001)
        let y = quantize(Double.random(in: minY..<maxY), scale: 0.001)
        cloud.appendPoint(x: x, y: y, z: z)
        let last = cloud.count - 1
        if cloud.classification != nil {
            cloud.classification![last] = 2
        }
        if cloud.returnNumber != nil {
            cloud.returnNumber![last] = 1
        }
        if cloud.numberOfReturns != nil {
            cloud.numberOfReturns![last] = 1
        }
    }
    return cloud
}

private func quantize(_ value: Double, scale: Double) -> Double {
    (value / scale).rounded() * scale
}
