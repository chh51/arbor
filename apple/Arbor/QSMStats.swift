// @file QSMStats.swift
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

/// Volume and cylinder count in one 2 cm radius class.
public struct RadiusBin: Sendable {
    /// R factor label, for example `(0,2]`. Radii are centimetres.
    public var radius: String
    public var volume: Double
    public var count: Int

    public init(radius: String, volume: Double, count: Int) {
        self.radius = radius
        self.volume = volume
        self.count = count
    }
}

/// Volume share of one branch order.
public struct OrderBin: Sendable {
    public var branchOrder: Int32
    public var volume: Double
    /// Cumulative share of total volume, percent, one decimal.
    public var percentage: Double

    public init(branchOrder: Int32, volume: Double, percentage: Double) {
        self.branchOrder = branchOrder
        self.volume = volume
        self.percentage = percentage
    }
}

/// One row of the R `stats_global` table.
public struct GlobalStats: Sendable {
    public var xRoot: Double
    public var yRoot: Double
    public var zRoot: Double
    public var xBH: Double
    public var yBH: Double
    public var zBH: Double
    public var dbh: Double
    public var volume: Double
    public var height: Double

    public init(xRoot: Double, yRoot: Double, zRoot: Double, xBH: Double, yBH: Double, zBH: Double, dbh: Double, volume: Double, height: Double) {
        self.xRoot = xRoot
        self.yRoot = yRoot
        self.zRoot = zRoot
        self.xBH = xBH
        self.yBH = yBH
        self.zBH = zBH
        self.dbh = dbh
        self.volume = volume
        self.height = height
    }
}

/// Stem diameter along the main axis. R column name: `diametre`.
public struct StemTaper: Sendable {
    public var distToRoot: Double
    public var diameter: Double

    public init(distToRoot: Double, diameter: Double) {
        self.distToRoot = distToRoot
        self.diameter = diameter
    }
}

/// The list returned by `qsm_stats` for one model.
public struct QSMStats: Sendable {
    public var statsByOrder: [OrderBin]
    public var statsByRadius: [RadiusBin]
    public var statsGlobal: GlobalStats
    public var stemTaper: [StemTaper]
    /// Short log labels from ``qsmMessage(_:short:)``.
    public var note: [String]

    public init(statsByOrder: [OrderBin], statsByRadius: [RadiusBin], statsGlobal: GlobalStats, stemTaper: [StemTaper], note: [String]) {
        self.statsByOrder = statsByOrder
        self.statsByRadius = statsByRadius
        self.statsGlobal = statsGlobal
        self.stemTaper = stemTaper
        self.note = note
    }
}

/// One row of `qsm_stats` on a forest: tree id, global metrics, and joined notes.
public struct QSFTreeStats: Sendable {
    public var treeID: Int?
    public var global: GlobalStats
    /// Log labels joined with ` + `.
    public var note: String

    public init(treeID: Int?, global: GlobalStats, note: String) {
        self.treeID = treeID
        self.global = global
        self.note = note
    }
}

/// Summary statistics for one model.
///
/// Volume by 2 cm radius class, volume by branch order, the stem taper of axis 1,
/// and global DBH, volume, and height. Cylinder `volume` is written as a side effect.
/// `display` is accepted so the signature matches R. This function does not draw plots.
///
/// - Parameter breastHeight: Passed to ``qsmDbh(_:breastHeight:)``. Default 1.30.
/// - Returns: `nil` when the model has no cylinders.
///
/// ## See Also
/// ``qsmDbh(_:breastHeight:)``
/// ``qsm(_:params:)``
public func qsmStats(_ model: QSM, breastHeight: Double = 1.30, display: Bool = false) throws -> QSMStats? {
    _ = display
    guard !model.cylinders.isEmpty else { return nil }
    let totalVolume = qsmVolume(model)
    let radiiCM = model.cylinders.map { $0.radius * 100 }
    let bins = radiusBreaks(maximum: radiiCM.max() ?? 0)
    var radiusVolume = [Double](repeating: 0, count: max(bins.count - 1, 0))
    var radiusCount = [Int](repeating: 0, count: radiusVolume.count)
    for (index, cylinder) in model.cylinders.enumerated() {
        guard let bin = radiusBin(radiiCM[index], breaks: bins) else { continue }
        radiusVolume[bin] += cylinder.volume
        radiusCount[bin] += 1
    }
    let statsByRadius = (0..<radiusVolume.count).compactMap { index -> RadiusBin? in
        guard radiusCount[index] > 0 else { return nil }
        let label = "(\(formatBreak(bins[index])),\(formatBreak(bins[index + 1]))]"
        return RadiusBin(radius: label, volume: round3(radiusVolume[index]), count: radiusCount[index])
    }

    var orderVolume: [Int32: Double] = [:]
    for cylinder in model.cylinders {
        orderVolume[cylinder.branchOrder, default: 0] += cylinder.volume
    }
    let orders = orderVolume.keys.sorted()
    let roundedOrders = orders.map { round3(orderVolume[$0] ?? 0) }
    let roundedTotal = roundedOrders.reduce(0, +)
    var running = 0.0
    let statsByOrder = zip(orders, roundedOrders).map { order, volume -> OrderBin in
        running += volume
        let percentage = roundedTotal == 0 ? 0 : round1(running / roundedTotal * 100)
        return OrderBin(branchOrder: order, volume: volume, percentage: percentage)
    }

    let stem = model.cylinders.filter { $0.axisID == 1 }
    guard let root = stem.first else {
        throw ArborError.invalidArgument("QSM has no cylinder on axis 1")
    }
    let maxSubtree = stem.map(\.subtreeLength).max() ?? 0
    let stemTaper = stem.map { cylinder in
        StemTaper(distToRoot: maxSubtree - cylinder.subtreeLength, diameter: cylinder.radius * 2)
    }.sorted { $0.distToRoot < $1.distToRoot }

    let measure = try qsmDbh(model, breastHeight: breastHeight)
    let global = GlobalStats(
        xRoot: root.startX,
        yRoot: root.startY,
        zRoot: root.startZ,
        xBH: measure.x,
        yBH: measure.y,
        zBH: measure.z,
        dbh: measure.dbh,
        volume: round3(totalVolume),
        height: round3((model.cylinders.map(\.endZ).max() ?? root.startZ) - root.startZ)
    )
    return QSMStats(
        statsByOrder: statsByOrder,
        statsByRadius: statsByRadius,
        statsGlobal: global,
        stemTaper: stemTaper,
        note: qsmMessage(model, short: true)
    )
}

/// Global statistics for each model in a forest.
///
/// Models with no cylinders are skipped. Returns `nil` when none remain.
/// `display` is ignored. R prints a notice in that case and still returns the table.
public func qsmStats(_ forest: QSF, breastHeight: Double = 1.30, display: Bool = false) throws -> [QSFTreeStats]? {
    _ = display
    var rows: [QSFTreeStats] = []
    for model in forest.models {
        guard let stats = try qsmStats(model, breastHeight: breastHeight, display: false) else { continue }
        rows.append(QSFTreeStats(
            treeID: model.id,
            global: stats.statsGlobal,
            note: stats.note.joined(separator: " + ")
        ))
    }
    return rows.isEmpty ? nil : rows
}

private func radiusBreaks(maximum: Double) -> [Double] {
    var breaks: [Double] = []
    var edge = 0.0
    let upper = maximum + 2
    while edge <= upper + 1e-9 {
        breaks.append(edge)
        edge += 2
    }
    if breaks.count < 2 {
        breaks = [0, 2]
    }
    return breaks
}

private func radiusBin(_ radiusCM: Double, breaks: [Double]) -> Int? {
    guard breaks.count >= 2, radiusCM > breaks[0], radiusCM <= breaks[breaks.count - 1] else { return nil }
    for index in 0..<(breaks.count - 1) where radiusCM > breaks[index] && radiusCM <= breaks[index + 1] {
        return index
    }
    return nil
}

private func formatBreak(_ value: Double) -> String {
    String(Int(value.rounded()))
}

private func round3(_ value: Double) -> Double {
    (value * 1000).rounded() / 1000
}

private func round1(_ value: Double) -> Double {
    (value * 10).rounded() / 10
}
