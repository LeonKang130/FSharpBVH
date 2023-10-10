open FSharpObj
open FSharpBVH
open System.Diagnostics
open System.Numerics

[<EntryPoint>]
let main _ =
    let stopwatch = Stopwatch()
    stopwatch.Start()
    let mesh = Parsing.ParseOBJ "stanford-bunny.obj"
    printfn "%A" stopwatch.Elapsed.TotalSeconds
    printfn "%A" (mesh.triangles.Length / 3)
    let aabbs =
        [|
            for i in 0 .. 3 .. mesh.triangles.Length - 1 ->
                let p1 = mesh.vertices[mesh.triangles[i]].position
                let p2 = mesh.vertices[mesh.triangles[i + 1]].position
                let p3 = mesh.vertices[mesh.triangles[i + 2]].position
                {
                    pMin = Vector3.Min(p1, Vector3.Min(p2, p3))
                    pMax = Vector3.Max(p1, Vector3.Max(p2, p3))
                }
        |]
    let nodes, primitives = Constructing.BuildBVH aabbs
    printfn "%A" stopwatch.Elapsed.TotalSeconds
    0
