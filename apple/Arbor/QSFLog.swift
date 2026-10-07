// @file QSFLog.swift
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

/// One grouped log line from ``qsfLog(_:)``.
///
/// `key` is the R list name: the code when the message has one, otherwise the label.
/// `index` is the zero-based position of each matching model in the forest.
public struct QSFLogEntry: Sendable {
    public var key: String
    public var code: String?
    /// `"warning"`, `"message"`, `"error"`, or `nil`.
    public var type: String?
    public var label: String?
    public var message: String
    public var index: [Int]
    public var treeID: [Int]

    public init(key: String, code: String?, type: String?, label: String?, message: String, index: [Int], treeID: [Int]) {
        self.key = key
        self.code = code
        self.type = type
        self.label = label
        self.message = message
        self.index = index
        self.treeID = treeID
    }
}

/// Structured log of every model in a forest.
///
/// ``subscript(_:)`` is the Swift form of `logs$W2`. `description` matches `print.qsf_log`.
public struct QSFLog: Sendable, CustomStringConvertible {
    public var entries: [QSFLogEntry]

    public init(entries: [QSFLogEntry]) {
        self.entries = entries
    }

    public subscript(key: String) -> QSFLogEntry? {
        entries.first { $0.key == key }
    }

    public var description: String {
        if entries.isEmpty {
            return "<qsf_log: no messages>\n"
        }
        return entries.map { entry in
            let code = entry.code ?? "--"
            let type = pad(entry.type ?? "unknown", to: 8)
            let label = pad(entry.label ?? entry.key, to: 30)
            return "[\(code)] \(type) \(label) n = \(entry.treeID.count) trees"
        }.joined(separator: "\n") + "\n"
    }
}

/// Group the log messages recorded while each tree was reconstructed.
///
/// A coded message looks like `[W0] [No wood point] This tree has no point labelled as wood`.
/// The letter is `W` (warning), `M` (message), or `E` (error). Messages written before
/// that convention have no code and are grouped by their short label.
///
/// - Returns: One entry per distinct code, or per label when there is no code.
///   An empty forest log is an empty ``QSFLog``.
///
/// ## See Also
/// ``qsfFilter(_:code:type:invert:flag:dbh:height:)``
/// ``qsmMessage(_:short:)``
public func qsfLog(_ forest: QSF) -> QSFLog {
    struct Row {
        var index: Int
        var treeID: Int?
        var code: String?
        var type: String?
        var label: String?
        var message: String
    }
    var rows: [Row] = []
    for (index, model) in forest.models.enumerated() where !model.message.isEmpty {
        for message in model.message {
            let parsed = parseQSFMessage(message)
            rows.append(Row(
                index: index,
                treeID: model.id,
                code: parsed.code,
                type: parsed.type,
                label: parsed.label,
                message: parsed.message
            ))
        }
    }
    if rows.isEmpty {
        return QSFLog(entries: [])
    }
    var order: [String] = []
    var groups: [String: [Row]] = [:]
    for row in rows {
        let key = row.code ?? row.label ?? row.message
        if groups[key] == nil {
            order.append(key)
            groups[key] = []
        }
        groups[key, default: []].append(row)
    }
    let entries = order.compactMap { key -> QSFLogEntry? in
        guard let matches = groups[key], let first = matches.first else { return nil }
        var indices: [Int] = []
        var ids: [Int] = []
        for match in matches {
            if !indices.contains(match.index) {
                indices.append(match.index)
            }
            if let id = match.treeID, !ids.contains(id) {
                ids.append(id)
            }
        }
        return QSFLogEntry(
            key: key,
            code: first.code,
            type: first.type,
            label: first.label,
            message: first.message,
            index: indices,
            treeID: ids
        )
    }
    return QSFLog(entries: entries)
}

struct ParsedQSFMessage {
    var code: String?
    var type: String?
    var label: String?
    var message: String
}

func parseQSFMessage(_ message: String) -> ParsedQSFMessage {
    let codes = [
        "W0": "No wood point",
        "E0": "QSM Generation Failed",
        "W2": "No valid measure",
        "W3": "Small tree allometry",
        "M0": "Broken tree",
        "W5": "Diameter anomaly"
    ]
    var rest = message[...]
    var code: String?
    if let bracket = takeBracket(&rest), isLogCode(bracket) {
        code = String(bracket)
    }
    var label: String?
    if let bracket = takeBracket(&rest) {
        label = String(bracket)
    } else if let code, let known = codes[code] {
        label = known
    }
    let type: String?
    switch code?.first {
    case "W": type = "warning"
    case "M": type = "message"
    case "E": type = "error"
    default: type = nil
    }
    return ParsedQSFMessage(code: code, type: type, label: label, message: message)
}

private func takeBracket(_ text: inout Substring) -> Substring? {
    let trimmed = text.drop { $0 == " " || $0 == "\t" }
    guard trimmed.first == "[" else {
        text = trimmed
        return nil
    }
    guard let end = trimmed.firstIndex(of: "]") else { return nil }
    let inner = trimmed[trimmed.index(after: trimmed.startIndex)..<end]
    text = trimmed[trimmed.index(after: end)...]
    return inner
}

private func isLogCode(_ text: Substring) -> Bool {
    guard let letter = text.first, letter == "E" || letter == "W" || letter == "M" else { return false }
    let digits = text.dropFirst()
    return !digits.isEmpty && digits.allSatisfy(\.isNumber)
}

private func pad(_ text: String, to width: Int) -> String {
    guard text.count < width else { return text }
    return text + String(repeating: " ", count: width - text.count)
}
