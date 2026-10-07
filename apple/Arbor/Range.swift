// @file Range.swift
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

/// One sensor position along a trajectory.
public struct TrajectorySample: Sendable {
    public var gpsTime: Double
    public var x: Double
    public var y: Double
    public var z: Double

    public init(gpsTime: Double, x: Double, y: Double, z: Double) {
        self.gpsTime = gpsTime
        self.x = x
        self.y = y
        self.z = z
    }
}

/// Sensor path used by ``addRange(_:trajectory:)``.
///
/// Swift form of the `sf` point object returned by R's `read_trajectory`.
public struct Trajectory: Sendable {
    public var samples: [TrajectorySample]
    public var crs: String

    public init(samples: [TrajectorySample], crs: String = "") {
        self.samples = samples
        self.crs = crs
    }
}

/// Read an ASCII trajectory.
///
/// The first column is GPS time and is renamed `gpstime`. The file must also
/// have columns named `x`, `y`, and `z`. When `dt` is positive, the first sample
/// in each `floor(gpstime / dt)` bucket is kept.
///
/// - Parameter file: Path to the trajectory file.
/// - Parameter dt: Seconds between kept samples. Default 0.5. Zero or negative keeps every row.
/// - Returns: Samples in file order, after the time bucket filter.
public func readTrajectory(_ file: String, dt: Double = 0.5) throws -> Trajectory {
    let text: String
    do {
        text = try String(contentsOfFile: file, encoding: .utf8)
    } catch {
        throw ArborError.invalidArgument("Cannot read trajectory file \(file)")
    }
    let lines = text.split(whereSeparator: \.isNewline).map { String($0).trimmingCharacters(in: .whitespaces) }.filter { !$0.isEmpty && !$0.hasPrefix("#") }
    guard let headerLine = lines.first else {
        throw ArborError.invalidArgument("Trajectory file must contain at least gpstime, x, y and z columns")
    }
    let delimiter: Character = headerLine.contains(",") ? "," : " "
    func fields(_ line: String) -> [String] {
        if delimiter == "," {
            return line.split(separator: ",", omittingEmptySubsequences: false).map { $0.trimmingCharacters(in: .whitespaces) }
        }
        return line.split(whereSeparator: \.isWhitespace).map(String.init)
    }
    var header = fields(headerLine)
    guard header.count >= 4 else {
        throw ArborError.invalidArgument("Trajectory file must contain at least gpstime, x, y and z columns")
    }
    let headerIsText = header.contains { Double($0) == nil }
    guard headerIsText else {
        throw ArborError.invalidArgument("Trajectory file must name columns x, y, and z")
    }
    header[0] = "gpstime"
    guard let xColumn = header.firstIndex(of: "x"),
          let yColumn = header.firstIndex(of: "y"),
          let zColumn = header.firstIndex(of: "z") else {
        throw ArborError.invalidArgument("Trajectory file must name columns x, y, and z")
    }
    var samples: [TrajectorySample] = []
    var seen: Set<Int> = []
    for line in lines.dropFirst() {
        let columns = fields(line)
        guard columns.count > max(xColumn, max(yColumn, zColumn)),
              let time = Double(columns[0]),
              let x = Double(columns[xColumn]),
              let y = Double(columns[yColumn]),
              let z = Double(columns[zColumn]) else {
            continue
        }
        if dt > 0 {
            let bucket = Int(floor(time / dt))
            if seen.contains(bucket) { continue }
            seen.insert(bucket)
        }
        samples.append(TrajectorySample(gpsTime: time, x: x, y: y, z: z))
    }
    return Trajectory(samples: samples)
}

/// Distance from each point to the sensor.
///
/// Interpolates the trajectory at the point's `gpstime` and stores the Euclidean
/// distance in `range`.
///
/// - Parameter cloud: A cloud with `gpstime`.
/// - Parameter trajectory: Samples from ``readTrajectory(_:dt:)``.
/// - Returns: The same cloud, with `range` written in place.
public func addRange(_ cloud: PointCloud, trajectory: Trajectory) throws -> PointCloud {
    try cloud.validate()
    guard let times = cloud.gpsTime else { throw ArborError.missingAttribute("gpstime") }
    let samples = trajectory.samples.sorted { $0.gpsTime < $1.gpsTime }
    guard !samples.isEmpty else {
        throw ArborError.invalidArgument("trajectory has no samples")
    }
    var range = [Double](repeating: 0, count: cloud.count)
    for index in 0..<cloud.count {
        let position = interpolate(time: times[index], samples: samples)
        let dx = cloud.x[index] - position.x
        let dy = cloud.y[index] - position.y
        let dz = cloud.z[index] - position.z
        range[index] = (dx * dx + dy * dy + dz * dz).squareRoot()
    }
    cloud.range = range
    return cloud
}

/// Drop points that are far from the sensor and low to the ground.
///
/// A point is removed only when it is farther than `distance` from the sensor and
/// lower than `hagMax` above ground. Canopy points beyond `distance` stay.
///
/// - Parameter cloud: A cloud with `hag` and `range`.
/// - Parameter distance: Metres. Default 15.
/// - Parameter hagMax: Metres. Default 4.
/// - Returns: A new cloud. The input is unchanged.
public func filterRange(_ cloud: PointCloud, distance: Double = 15, hagMax: Double = 4) throws -> PointCloud {
    try cloud.validate()
    let hag = try cloud.requireHag()
    guard let range = cloud.range else { throw ArborError.missingAttribute("range") }
    let keep = (0..<cloud.count).map { index in
        hag[index] >= hagMax || range[index] <= distance
    }
    return cloud.selecting(keep)
}

private func interpolate(time: Double, samples: [TrajectorySample]) -> TrajectorySample {
    if time <= samples[0].gpsTime { return samples[0] }
    if time >= samples[samples.count - 1].gpsTime { return samples[samples.count - 1] }
    var low = 0
    var high = samples.count - 1
    while high - low > 1 {
        let mid = (low + high) / 2
        if samples[mid].gpsTime <= time {
            low = mid
        } else {
            high = mid
        }
    }
    let left = samples[low]
    let right = samples[high]
    let span = right.gpsTime - left.gpsTime
    let weight = span == 0 ? 0 : (time - left.gpsTime) / span
    return TrajectorySample(
        gpsTime: time,
        x: left.x + (right.x - left.x) * weight,
        y: left.y + (right.y - left.y) * weight,
        z: left.z + (right.z - left.z) * weight
    )
}
