// @file QSFFilter.swift
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

/// Keep the models that match every supplied criterion.
///
/// Criteria combine with AND. `code` and `type` keep models that carry a matching
/// log entry, unless `invert` is true, in which case those models are dropped.
/// An entry must match both when both are set. `flag` true keeps models with any
/// log entry. `flag` false keeps models with none. `dbh` and `height` are
/// `c(min, max)` in metres and must contain two values.
///
/// - Parameter code: Log codes such as `W0` or `E1`.
/// - Parameter type: `"warning"`, `"message"`, or `"error"`.
/// - Parameter invert: Drop matches instead of keeping them. Default false.
/// - Parameter flag: Filter on whether a log entry exists at all.
/// - Parameter dbh: Inclusive DBH range, metres.
/// - Parameter height: Inclusive height range, metres.
/// - Returns: A forest of the selected models.
///
/// ## See Also
/// ``qsfLog(_:)``
public func qsfFilter(
    _ forest: QSF,
    code: [String]? = nil,
    type: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    var keep = Array(repeating: true, count: forest.count)
    var log: QSFLog?
    if code != nil || type != nil {
        log = qsfLog(forest)
        let flagged = logMatches(log!, code: code, type: type, count: forest.count)
        for index in keep.indices {
            keep[index] = keep[index] && (invert ? !flagged[index] : flagged[index])
        }
    }
    if let flag {
        if log == nil {
            log = qsfLog(forest)
        }
        let any = logMatches(log!, code: nil, type: nil, count: forest.count)
        for index in keep.indices {
            keep[index] = keep[index] && (flag ? any[index] : !any[index])
        }
    }
    if let dbh {
        guard dbh.count == 2 else {
            throw ArborError.invalidArgument("`dbh` must be a numeric vector of length 2: c(min, max).")
        }
        for (index, model) in forest.models.enumerated() {
            let measure = try qsmDbh(model)
            keep[index] = keep[index] && measure.dbh >= dbh[0] && measure.dbh <= dbh[1]
        }
    }
    if let height {
        guard height.count == 2 else {
            throw ArborError.invalidArgument("`height` must be a numeric vector of length 2: c(min, max).")
        }
        let values = qsmHeight(forest)
        for index in keep.indices {
            guard let value = values[index] else {
                keep[index] = false
                continue
            }
            keep[index] = keep[index] && value >= height[0] && value <= height[1]
        }
    }
    return forest[keep]
}

/// Models with no log entry.
public func qsfFilterOk(
    _ forest: QSF,
    code: [String]? = nil,
    type: [String]? = nil,
    invert: Bool = false,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: code, type: type, invert: invert, flag: false, dbh: dbh, height: height)
}

/// Models marked as saplings (`W3`). Arbor does not measure these on purpose.
public func qsfFilterSapling(
    _ forest: QSF,
    type: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: ["W3"], type: type, invert: invert, flag: flag, dbh: dbh, height: height)
}

/// Models where measurement was attempted and nothing valid was found (`W2`).
///
/// These are not saplings. Saplings are ``qsfFilterSapling(_:type:invert:flag:dbh:height:)``.
public func qsfFilterNomeasure(
    _ forest: QSF,
    type: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: ["W2"], type: type, invert: invert, flag: flag, dbh: dbh, height: height)
}

/// Models flagged with code `W4`.
public func qsfFilterBroken(_ forest: QSF) throws -> QSF {
    try qsfFilter(forest, code: ["W4"])
}

/// Models with at least one log entry.
///
/// When `code` or `type` is set, the filter uses that match instead of "any entry".
public func qsfFilterFlagged(
    _ forest: QSF,
    code: [String]? = nil,
    type: [String]? = nil,
    invert: Bool = false,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    if code == nil && type == nil {
        return try qsfFilter(forest, invert: invert, flag: true, dbh: dbh, height: height)
    }
    return try qsfFilter(forest, code: code, type: type, invert: invert, dbh: dbh, height: height)
}

/// Models that carry a warning. `code` narrows that to one warning.
public func qsfFilterWarnings(
    _ forest: QSF,
    code: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: code, type: ["warning"], invert: invert, flag: flag, dbh: dbh, height: height)
}

/// Models that carry an error. `code` narrows that to one error.
public func qsfFilterErrors(
    _ forest: QSF,
    code: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: code, type: ["error"], invert: invert, flag: flag, dbh: dbh, height: height)
}

/// Models that carry a plain message. `code` narrows that to one message.
public func qsfFilterMessages(
    _ forest: QSF,
    code: [String]? = nil,
    invert: Bool = false,
    flag: Bool? = nil,
    dbh: [Double]? = nil,
    height: [Double]? = nil
) throws -> QSF {
    try qsfFilter(forest, code: code, type: ["message"], invert: invert, flag: flag, dbh: dbh, height: height)
}

private func logMatches(_ log: QSFLog, code: [String]?, type: [String]?, count: Int) -> [Bool] {
    var matched = Array(repeating: false, count: count)
    for entry in log.entries {
        let codeMatches = code == nil || (entry.code.map { code!.contains($0) } ?? false)
        let typeMatches = type == nil || (entry.type.map { type!.contains($0) } ?? false)
        guard codeMatches && typeMatches else { continue }
        for index in entry.index where index >= 0 && index < count {
            matched[index] = true
        }
    }
    return matched
}
