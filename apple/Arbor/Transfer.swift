// @file Transfer.swift
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

/// Copy attributes from a sparse cloud onto a dense cloud.
///
/// Each point in `destination` receives the value of its nearest neighbour in
/// `source`. Names are the R column names (`treeID`, `foliage`, `UserData`,
/// `hag`, `pwood`, `Classification`, `range`, `gpstime`, `R`, `G`, `B`,
/// `ReturnNumber`, `NumberOfReturns`, and `X`, `Y`, `Z`).
///
/// - Parameter source: Cloud that already has `attributes`.
/// - Parameter destination: Cloud that receives them.
/// - Parameter attributes: Column names to copy.
/// - Returns: `destination`, updated in place.
public func transferAttributes(from source: PointCloud, to destination: PointCloud, attributes: [String]) throws -> PointCloud {
    try source.validate()
    try destination.validate()
    guard source.count > 0 else {
        throw ArborError.invalidArgument("source point cloud is empty")
    }
    for name in attributes {
        guard source.hasColumn(name) else {
            throw ArborError.missingAttribute(name)
        }
    }
    let nearest = nearestIndices(from: source, to: destination)
    for name in attributes {
        try source.copyColumn(name, to: destination, index: nearest)
    }
    return destination
}

private struct Cell: Hashable {
    var x: Int
    var y: Int
    var z: Int
}

private func nearestIndices(from source: PointCloud, to destination: PointCloud) -> [Int] {
    var minX = source.x[0]
    var minY = source.y[0]
    var minZ = source.z[0]
    var maxX = minX
    var maxY = minY
    var maxZ = minZ
    for index in 0..<source.count {
        minX = min(minX, source.x[index])
        minY = min(minY, source.y[index])
        minZ = min(minZ, source.z[index])
        maxX = max(maxX, source.x[index])
        maxY = max(maxY, source.y[index])
        maxZ = max(maxZ, source.z[index])
    }
    let volume = max(maxX - minX, 1e-6) * max(maxY - minY, 1e-6) * max(maxZ - minZ, 1e-6)
    let cell = max(pow(volume / Double(source.count), 1.0 / 3.0), 1e-3)
    var bins: [Cell: [Int]] = [:]
    func cellFor(_ x: Double, _ y: Double, _ z: Double) -> Cell {
        Cell(
            x: Int(floor((x - minX) / cell)),
            y: Int(floor((y - minY) / cell)),
            z: Int(floor((z - minZ) / cell))
        )
    }
    for index in 0..<source.count {
        bins[cellFor(source.x[index], source.y[index], source.z[index]), default: []].append(index)
    }

    return (0..<destination.count).map { query in
        let qx = destination.x[query]
        let qy = destination.y[query]
        let qz = destination.z[query]
        let home = cellFor(qx, qy, qz)
        var best = 0
        var bestDistance = Double.greatestFiniteMagnitude
        var radius = 0
        while true {
            for iz in (home.z - radius)...(home.z + radius) {
                for iy in (home.y - radius)...(home.y + radius) {
                    for ix in (home.x - radius)...(home.x + radius) {
                        let onShell = radius == 0 || abs(ix - home.x) == radius || abs(iy - home.y) == radius || abs(iz - home.z) == radius
                        guard onShell, let bucket = bins[Cell(x: ix, y: iy, z: iz)] else { continue }
                        for index in bucket {
                            let dx = source.x[index] - qx
                            let dy = source.y[index] - qy
                            let dz = source.z[index] - qz
                            let distance = dx * dx + dy * dy + dz * dz
                            if distance < bestDistance {
                                bestDistance = distance
                                best = index
                            }
                        }
                    }
                }
            }
            let minXBound = minX + Double(home.x - radius) * cell
            let maxXBound = minX + Double(home.x + radius + 1) * cell
            let minYBound = minY + Double(home.y - radius) * cell
            let maxYBound = minY + Double(home.y + radius + 1) * cell
            let minZBound = minZ + Double(home.z - radius) * cell
            let maxZBound = minZ + Double(home.z + radius + 1) * cell
            let outside = min(
                min(qx - minXBound, maxXBound - qx),
                min(min(qy - minYBound, maxYBound - qy), min(qz - minZBound, maxZBound - qz))
            )
            if outside >= 0, bestDistance <= outside * outside {
                break
            }
            radius += 1
            if radius > source.count { break }
        }
        if bestDistance.isInfinite {
            var fallback = 0
            var fallbackDistance = Double.greatestFiniteMagnitude
            for index in 0..<source.count {
                let dx = source.x[index] - qx
                let dy = source.y[index] - qy
                let dz = source.z[index] - qz
                let distance = dx * dx + dy * dy + dz * dz
                if distance < fallbackDistance {
                    fallbackDistance = distance
                    fallback = index
                }
            }
            return fallback
        }
        return best
    }
}

private extension PointCloud {
    func hasColumn(_ name: String) -> Bool {
        switch name {
        case "X", "Y", "Z": return true
        case "Classification": return classification != nil
        case "hag": return hag != nil
        case "pwood": return pwood != nil
        case "foliage": return foliage != nil
        case "passage": return passage != nil
        case "UserData": return userData != nil
        case "treeID": return treeID != nil
        case "range": return range != nil
        case "gpstime": return gpsTime != nil
        case "R": return red != nil
        case "G": return green != nil
        case "B": return blue != nil
        case "ReturnNumber": return returnNumber != nil
        case "NumberOfReturns": return numberOfReturns != nil
        default: return false
        }
    }

    func copyColumn(_ name: String, to destination: PointCloud, index: [Int]) throws {
        switch name {
        case "X":
            destination.x = index.map { x[$0] }
        case "Y":
            destination.y = index.map { y[$0] }
        case "Z":
            destination.z = index.map { z[$0] }
        case "Classification":
            destination.classification = index.map { classification![$0] }
        case "hag":
            destination.hag = index.map { hag![$0] }
        case "pwood":
            destination.pwood = index.map { pwood![$0] }
        case "foliage":
            destination.foliage = index.map { foliage![$0] }
        case "passage":
            destination.passage = index.map { passage![$0] }
        case "UserData":
            destination.userData = index.map { userData![$0] }
        case "treeID":
            destination.treeID = index.map { treeID![$0] }
        case "range":
            destination.range = index.map { range![$0] }
        case "gpstime":
            destination.gpsTime = index.map { gpsTime![$0] }
        case "R":
            destination.red = index.map { red![$0] }
        case "G":
            destination.green = index.map { green![$0] }
        case "B":
            destination.blue = index.map { blue![$0] }
        case "ReturnNumber":
            destination.returnNumber = index.map { returnNumber![$0] }
        case "NumberOfReturns":
            destination.numberOfReturns = index.map { numberOfReturns![$0] }
        default:
            throw ArborError.missingAttribute(name)
        }
    }
}
