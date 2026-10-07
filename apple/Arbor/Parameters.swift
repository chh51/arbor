// @file Parameters.swift
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

/// Point flagged as a tree in `UserData`.
///
/// Usually every point of interest is `ARBORTREE`. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/).
public let ARBORTREE: Int32 = 0

/// Point below the ground cut, ignored by later Arbor stages.
public let ARBORLOW: Int32 = 1

/// Small tree excluded from QSM construction.
///
/// See ``flagSmallTrees(_:maxHeight:)``.
public let ARBORUNDERSTORY: Int32 = 2

/// Point whose tree sits in the plot buffer.
///
/// See ``flagBuffer(_:seeds:buffer:)``.
public let ARBORBUFFER: Int32 = 3

/// Height cut shared by the pipeline stages.
public struct GlobalParameters: Sendable {
    /// Metres above ground. Points below this are `ARBORLOW`.
    public var cutAboveGround: Double

    public init(cutAboveGround: Double = 0.25) {
        self.cutAboveGround = cutAboveGround
    }
}

/// Neighbourhood size for ``woodLikelihood(_:params:)``.
public struct WoodLikelihoodParameters: Sendable {
    /// Number of neighbours used to estimate local anisotropy.
    public var k: Int

    public init(k: Int = 80) {
        self.k = k
    }
}

/// Graph used by semantic segmentation, seed finding, and instance segmentation.
public struct GraphParameters: Sendable {
    public var downward: Bool
    public var k: Int
    public var kSeed: Int
    public var decimation: Double
    public var spaceRes: Double
    public var maxGap: Double
    public var power: Double
    public var woodToWood: Double
    public var leafToLeaf: Double
    public var woodToLeaf: Double
    /// Cost multiplier for a turn of 0...180 degrees. Index 181 is unused.
    public var anglePenalty: [Float]

    public init(
        downward: Bool = false,
        k: Int = 10,
        kSeed: Int = 100,
        decimation: Double = 0.05,
        spaceRes: Double = 0.2,
        maxGap: Double = 1.0,
        power: Double = 3.0,
        woodToWood: Double = 0.1,
        leafToLeaf: Double = 20.0,
        woodToLeaf: Double = 1000.0,
        anglePenalty: [Float]? = nil
    ) {
        self.downward = downward
        self.k = k
        self.kSeed = kSeed
        self.decimation = decimation
        self.spaceRes = spaceRes
        self.maxGap = maxGap
        self.power = power
        self.woodToWood = woodToWood
        self.leafToLeaf = leafToLeaf
        self.woodToLeaf = woodToLeaf
        self.anglePenalty = anglePenalty ?? GraphParameters.defaultAnglePenalty()
    }

    /// The C++ default: `exp(0.046051 * degrees)`, capped at 100 past 100 degrees.
    public static func defaultAnglePenalty() -> [Float] {
        (0...180).map { degrees in
            let raw = exp(0.046051 * Float(degrees))
            return degrees > 100 ? 100 : raw
        }
    }
}

/// Thresholds for wood and foliage classification.
public struct SemanticParameters: Sendable {
    public var minPassage: Int
    public var highPwoodThreshold: Double
    public var mediumPwoodThreshold: Double
    public var connectedComponentsRes: Double
    public var connectedComponentsMin: Int
    public var woodAssignationK: Int
    public var woodAssignationDist: Double
    public var woodExtraReasignationK: Int
    public var woodExtraReasignationDist: Double
    public var mediumPwoodSorK: Int
    public var mediumPwoodSorM: Double
    public var groundRes: Double

    public init(
        minPassage: Int = 3,
        highPwoodThreshold: Double = 0.9,
        mediumPwoodThreshold: Double = 0.75,
        connectedComponentsRes: Double = 0.05,
        connectedComponentsMin: Int = 2000,
        woodAssignationK: Int = 50,
        woodAssignationDist: Double = 0.05,
        woodExtraReasignationK: Int = 10,
        woodExtraReasignationDist: Double = 0.03,
        mediumPwoodSorK: Int = 50,
        mediumPwoodSorM: Double = 0.05,
        groundRes: Double = 0.2
    ) {
        self.minPassage = minPassage
        self.highPwoodThreshold = highPwoodThreshold
        self.mediumPwoodThreshold = mediumPwoodThreshold
        self.connectedComponentsRes = connectedComponentsRes
        self.connectedComponentsMin = connectedComponentsMin
        self.woodAssignationK = woodAssignationK
        self.woodAssignationDist = woodAssignationDist
        self.woodExtraReasignationK = woodExtraReasignationK
        self.woodExtraReasignationDist = woodExtraReasignationDist
        self.mediumPwoodSorK = mediumPwoodSorK
        self.mediumPwoodSorM = mediumPwoodSorM
        self.groundRes = groundRes
    }
}

/// Slice heights used to search for stem seeds.
public struct SeedParameters: Sendable {
    public var sliceAt: [Double]
    public var sliceThickness: Double
    public var minPassage: Int
    public var safeZone: Double

    public init(
        sliceAt: [Double] = [0.7, 0.9],
        sliceThickness: Double = 0.05,
        minPassage: Int = 15,
        safeZone: Double = 0.2
    ) {
        self.sliceAt = sliceAt
        self.sliceThickness = sliceThickness
        self.minPassage = minPassage
        self.safeZone = safeZone
    }
}

/// Controls applied during instance segmentation.
public struct InstanceParameters: Sendable {
    public var oversegmentationSolverEnabled: Bool

    public init(oversegmentationSolverEnabled: Bool = true) {
        self.oversegmentationSolverEnabled = oversegmentationSolverEnabled
    }
}

/// Controls for quantitative structure models.
public struct QSMParameters: Sendable {
    public var skeletonNodeDistance: Float
    public var dbscanEpsDistance: Float
    public var maxD: Float
    public var apexRadius: Float
    public var smoothSteps: Int
    public var minMeasurableDBH: Float
    public var minMeasurableRadius: Float
    public var brokenDetectionEnabled: Bool
    public var allometryScale: Float
    public var allometryName: String

    public init(
        skeletonNodeDistance: Float = 0.1,
        dbscanEpsDistance: Float = 0.1,
        maxD: Float = 0.1,
        apexRadius: Float = 0.0025,
        smoothSteps: Int = 15,
        minMeasurableDBH: Float = 0.05,
        minMeasurableRadius: Float = 0.025,
        brokenDetectionEnabled: Bool = true,
        allometryScale: Float = 1.0,
        allometryName: String = "Griese2025"
    ) {
        self.skeletonNodeDistance = skeletonNodeDistance
        self.dbscanEpsDistance = dbscanEpsDistance
        self.maxD = maxD
        self.apexRadius = apexRadius
        self.smoothSteps = smoothSteps
        self.minMeasurableDBH = minMeasurableDBH
        self.minMeasurableRadius = minMeasurableRadius
        self.brokenDetectionEnabled = brokenDetectionEnabled
        self.allometryScale = allometryScale
        self.allometryName = allometryName
    }
}

/// Every threshold the pipeline reads.
///
/// `arborParametersDefault` is this value. The R package deliberately does not
/// document each knob: the pipeline is meant to run with these defaults. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/).
public struct ArborParameters: Sendable {
    public var global: GlobalParameters
    public var woodLikelihood: WoodLikelihoodParameters
    /// R name: `pathfinder`.
    public var pathfinder: GraphParameters
    public var semantic: SemanticParameters
    public var seeds: SeedParameters
    public var instance: InstanceParameters
    public var qsm: QSMParameters

    public init(
        global: GlobalParameters = GlobalParameters(),
        woodLikelihood: WoodLikelihoodParameters = WoodLikelihoodParameters(),
        pathfinder: GraphParameters = GraphParameters(),
        semantic: SemanticParameters = SemanticParameters(),
        seeds: SeedParameters = SeedParameters(),
        instance: InstanceParameters = InstanceParameters(),
        qsm: QSMParameters = QSMParameters()
    ) {
        self.global = global
        self.woodLikelihood = woodLikelihood
        self.pathfinder = pathfinder
        self.semantic = semantic
        self.seeds = seeds
        self.instance = instance
        self.qsm = qsm
    }
}

/// Built-in parameters, matching `arbor_parameters_default` and `arbor::settings::ArborParameters`.
public let arborParametersDefault = ArborParameters()
