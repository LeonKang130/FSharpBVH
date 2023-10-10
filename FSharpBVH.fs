#nowarn "9"
namespace FSharpBVH
open System
open System.Numerics
open System.Collections.Generic

[<Struct>]
type AABB =
    {
        pMin: Vector3
        pMax: Vector3
    }

    static member Default =
        {
            pMin = Vector3 Single.PositiveInfinity
            pMax = Vector3 Single.NegativeInfinity
        }
[<Struct>]
type PayLoad =
    {
        parent: int
        offset: int
        length: int
    }
[<Struct>]
type BVHNode =
    {
        aabb: AABB
        payload: PayLoad
    }
[<Struct>]
type private BVHBuildBin =
    {
        mutable aabb: AABB
        mutable primitiveCount: int
    }
module Constructing =
    let inline surfaceArea (aabb: inref<AABB>) =
        let extents = aabb.pMax - aabb.pMin
        if extents.X < 0.0f || extents.Y < 0.0f || extents.Z < 0.0f then 0.0f
        else 2.0f * (extents.X * (extents.Y + extents.Z) + extents.Y * extents.Z)
    let inline splitDimension (aabb: inref<AABB>) =
        let extents = aabb.pMax - aabb.pMin
        if extents.X > extents.Y then if extents.X > extents.Z then 0 else 2
        elif extents.Y > extents.Z then 1 else 2
    let inline combineAABB (a: inref<AABB>) (b: inref<AABB>) =
        {
            pMin = Vector3.Min(a.pMin, b.pMin)
            pMax = Vector3.Max(a.pMax, b.pMax)
        }
    let inline calcCentroid (aabb: inref<AABB>) =
        0.5f * (aabb.pMax + aabb.pMin)
    
    let BuildBVH (aabbs: AABB array) =
        let numBins = 12
        let numPrimitives = aabbs.Length
        let mutable centroids = Array.zeroCreate<Vector3> numPrimitives
        for i in 0 .. numPrimitives - 1 do centroids[i] <- calcCentroid &aabbs[i]
        let payloads = Stack<PayLoad>(32)
        payloads.Push { parent = -1; offset = 0; length = numPrimitives }
        let mutable nodes = ResizeArray<BVHNode>(numPrimitives)
        let mutable primitives = [| 0 .. numPrimitives - 1 |]
        let bins = [| for _ in 0 .. numBins - 1 -> { aabb = AABB.Default; primitiveCount = 0 } |]
        let mutable costs = Array.zeroCreate<float32> (numBins - 1)
        while payloads.Count <> 0 do
            let payload = payloads.Pop()
            if payload.length = 1 then
                let primitive = primitives[payload.offset]
                nodes.Add { aabb = aabbs[primitive]; payload = payload }
            else
                let mutable aabb = aabbs[primitives[payload.offset]]
                for i in payload.offset + 1 .. payload.offset + payload.length - 1 do
                    aabb <- combineAABB &aabb &aabbs[primitives[i]]
                let dimension = splitDimension &aabb
                for i in payload.offset .. payload.offset + payload.length - 1 do
                    let primitive = primitives[i]
                    let percentage =
                        if dimension = 0 then
                            (centroids[primitive].X - aabb.pMin.X) / (aabb.pMax.X - aabb.pMin.X)
                        elif dimension = 1 then
                            (centroids[primitive].Y - aabb.pMin.Y) / (aabb.pMax.Y - aabb.pMin.Y)
                        else
                            (centroids[primitive].Z - aabb.pMin.Z) / (aabb.pMax.Z - aabb.pMin.Z)
                    let idx = min (numBins - 1) (int (float32 numBins * percentage))
                    bins[idx].aabb <- combineAABB &bins[idx].aabb &aabbs[primitive]
                    bins[idx].primitiveCount <- bins[idx].primitiveCount + 1
                let mutable box = bins[0].aabb
                let mutable primitiveCount = bins[0].primitiveCount
                costs[0] <- float32 primitiveCount * surfaceArea &box
                for i in 1 .. numBins - 2 do
                    box <- combineAABB &box &bins[i].aabb
                    primitiveCount <- primitiveCount + bins[i].primitiveCount
                    costs[i] <- float32 primitiveCount * surfaceArea &box
                box <- bins[numBins - 1].aabb
                primitiveCount <- bins[numBins - 1].primitiveCount
                costs[numBins - 2] <- costs[numBins - 2] + float32 primitiveCount * surfaceArea &box
                for i in numBins - 2 .. -1 .. 1 do
                    box <- combineAABB &box &bins[i].aabb
                    primitiveCount <- primitiveCount + bins[i].primitiveCount
                    costs[i - 1] <- costs[i - 1] + float32 primitiveCount * surfaceArea &box
                let minCost = Array.min costs
                let leftBins = 1 + (Array.IndexOf(costs, minCost))
                let mutable leftSegmentLength = 0
                for i in 0 .. leftBins - 1 do leftSegmentLength <- leftSegmentLength + bins[i].primitiveCount
                if leftSegmentLength = 0 || leftSegmentLength = payload.length then
                    nodes.Add { aabb = aabb; payload = payload }
                else
                    let mutable left = payload.offset
                    let mutable right = left + payload.length - 1
                    while left <> right do
                        let primitive = primitives[left]
                        let percentage =
                            if dimension = 0 then
                                (centroids[primitive].X - aabb.pMin.X) / (aabb.pMax.X - aabb.pMin.X)
                            elif dimension = 1 then
                                (centroids[primitive].Y - aabb.pMin.Y) / (aabb.pMax.Y - aabb.pMin.Y)
                            else
                                (centroids[primitive].Z - aabb.pMin.Z) / (aabb.pMax.Z - aabb.pMin.Z)
                        let idx = min (numBins - 1) (int (float32 numBins * percentage))
                        if idx < leftBins then left <- left + 1
                        else
                            let temp = primitives[right]
                            primitives[right] <- primitives[left]
                            primitives[left] <- temp
                            right <- right - 1
                    let current = nodes.Count
                    nodes.Add { aabb = aabb; payload = { parent = payload.parent; offset = 0; length = 0 } }
                    payloads.Push { payload with length = leftSegmentLength; parent = current }
                    payloads.Push { parent = current; offset = payload.offset + leftSegmentLength; length = payload.length - leftSegmentLength }
                for i in 0 .. numBins - 1 do
                    bins[i].primitiveCount <- 0
                    bins[i].aabb <- AABB.Default
        let mutable nodes = nodes.ToArray()
        for i in 0 .. nodes.Length - 1 do
            let parent = nodes[i].payload.parent
            if parent <> -1 && parent + 1 <> i then nodes[parent] <- { nodes[parent] with payload = { nodes[parent].payload with offset = i } }
        nodes, primitives