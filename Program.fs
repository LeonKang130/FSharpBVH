open FSharpObj
open FSharpBVH
open System
open System.Numerics
open FSharp.Collections.ParallelSeq

[<EntryPoint>]
let main _ =
    let mesh = Parsing.ParseOBJ "xyzrgb_dragon.obj"
    let aabbs =
        seq { 0 .. 3 .. mesh.triangles.Length - 1 }
        |> PSeq.map (fun i ->
            let p1 = mesh.vertices[mesh.triangles[i]].position
            let p2 = mesh.vertices[mesh.triangles[i + 1]].position
            let p3 = mesh.vertices[mesh.triangles[i + 2]].position
            {
                pMin = Vector3.Min(p1, Vector3.Min(p2, p3))
                pMax = Vector3.Max(p1, Vector3.Max(p2, p3))
            }
        )
        |> PSeq.toArray
    let nodes, primitives = Constructing.BuildBVH aabbs
    let mutable triangles = Array.zeroCreate mesh.triangles.Length
    primitives
    |> Array.Parallel.iteri (fun i primitive ->
        triangles[3 * i] <- mesh.triangles[3 * primitive]
        triangles[3 * i + 1] <- mesh.triangles[3 * primitive + 1]
        triangles[3 * i + 2] <- mesh.triangles[3 * primitive + 2]
    )
    let mesh = { mesh with triangles = triangles }
    0