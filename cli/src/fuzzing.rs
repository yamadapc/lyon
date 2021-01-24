use crate::commands::{FuzzCmd, Tessellator};
use lyon::extra::debugging::find_reduced_test_case;
use lyon::math::*;
use lyon::path::Path;
use lyon::path::traits::PathBuilder;
use lyon::tess2;
use lyon::tessellation::geometry_builder::{NoOutput, VertexBuffers, BuffersBuilder};
use lyon::tessellation::{FillTessellator, StrokeTessellator, FillOptions, FillVertex};
use lyon::algorithms::hatching::*;
use lyon::geom::LineSegment;
use rand;
use std::cmp::{max, min};


fn rasterize(path: &Path, options: &FillOptions) {
    use euc::*;

    let mut buffers: VertexBuffers<Vector, u16> = VertexBuffers::new();

    FillTessellator::new().tessellate(
        path,
        options,
        &mut BuffersBuilder::new(
            &mut buffers,
            |vertex: FillVertex| { vertex.position().to_vector() },
        ),
    ).unwrap();

    let mut triangles = Vec::new();
    for idx in &buffers.indices {
        triangles.push((buffers.vertices[*idx as usize]/ 1000.0 ).to_array());
    }

    struct SwRaster;
    impl Pipeline for SwRaster {
        type Vertex = [f32; 2];
        type VsOut = ();
        type Pixel = u8;

        #[inline(always)]
        fn get_depth_strategy(&self) -> DepthStrategy { DepthStrategy::None }

        #[inline(always)]
        fn vert(&self, pos: &Self::Vertex) -> ([f32; 4], Self::VsOut) {
            ([pos[0], -pos[1], 0.0, 1.0], ())
        }

        #[inline(always)]
        fn frag(&self, _: &Self::VsOut) -> Self::Pixel { 1 }
    }

    pub struct AdditiveBlend<T>(buffer::Buffer2d<T>);
    let mut color = AdditiveBlend(buffer::Buffer2d::new([1000, 1000], 0));

    impl<T: Clone + std::ops::Add<T, Output = T>> Target for AdditiveBlend<T> {
        type Item = T;

        #[inline(always)]
        fn size(&self) -> [usize; 2] { self.0.size() }

        #[inline(always)]
        unsafe fn set(&mut self, pos: [usize; 2], item: Self::Item) {
            self.0.set(pos, self.0.get(pos) + item);
        }

        #[inline(always)]
        unsafe fn get(&self, pos: [usize; 2]) -> Self::Item { self.0.get(pos) }

        fn clear(&mut self, fill: Self::Item) { self.0.clear(fill); }
    }

    SwRaster.draw::<rasterizer::Triangles<buffer::Buffer2d<f32>, rasterizer::BackfaceCullingDisabled>, _>(
        &triangles[..],
        &mut color,
        None,
    );

    let mut overlaps = Vec::new();
    'outer: for y in 0..1000 {
        for x in 0..1000 {
            let px = unsafe { color.get([x, y]) };
            if px > 1 {
                println!("overlap at {:?} {:?} -> {:?}", x, y, px);
                overlaps.push([x, y]);
                //break 'outer;
            }
        }
    }

    if overlaps.len() > 0 {
        use std::path::Path;
        use std::fs::File;
        use std::io::BufWriter;

        let path = Path::new(r"tmp.png");
        let file = File::create(path).unwrap();
        let ref mut w = BufWriter::new(file);
        let mut encoder = png::Encoder::new(w, 1000, 1000);
        encoder.set_color(png::ColorType::RGBA);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().unwrap();

        let mut data = vec![255; 1000*1000*4];
        for y in 0..1000 {
            for x in 0..1000 {
                let pixel = unsafe { color.get([x, y]) } as u8;

                let color = match pixel {
                    0 => [0, 0, 0, 255],
                    1 => [255, 255, 255, 255],
                    2 => [150, 2, 0, 255],
                    n => [255, n, 0, 255],
                };

                let index = y * 1000 + x;
                data[index * 4] = color[0];
                data[index * 4 + 1] = color[1];
                data[index * 4 + 2] = color[2];
                data[index * 4 + 3] = color[3];
            }
        }

        for overlap in &overlaps {
            println!("overlap at {:?}", overlap);
            for x in (overlap[0] - 5) .. (overlap[0] + 5) {
                let y = overlap[1] - 5;
                let idx = y * 1000 + x;
                data[idx * 4] = 255 - data[idx * 4];

                let y = overlap[1] + 5;
                let idx = y * 1000 + x;
                data[idx * 4] = 255 - data[idx * 4];
            }

            for y in (overlap[1] - 5) .. (overlap[1] + 5) {
                let x = overlap[0] - 5;
                let idx = y * 1000 + x;
                data[idx * 4] = 255 - data[idx * 4];

                let x = overlap[0] + 5;
                let idx = y * 1000 + x;
                data[idx * 4] = 255 - data[idx * 4];
            }
        }

        println!("triangles: {:?}", triangles);

        writer.write_image_data(&data).unwrap();

        panic!("{:?} overlapping pixels", overlaps);
    }
}

fn random_point() -> Point {
    point(
        rand::random::<f32>() * 1000.0,
        rand::random::<f32>() * 1000.0,
    )
}

fn generate_path(cmd: &FuzzCmd, iteration: u64) -> Path {
    let mut path = Path::builder();

    let min_points = cmd.min_points.unwrap_or(5);
    let max_points = max(min_points, cmd.max_points.unwrap_or(5_000));
    let diff = max_points - min_points;

    let target = min_points + min(diff, (iteration / 5000) as u32);

    let mut num_points = 0;
    loop {
        let num_cmds = 3 + rand::random::<u32>() % (target - num_points);

        path.begin(random_point());
        num_points += 1;
        for _ in 0..num_cmds {
            path.line_to(random_point());
            num_points += 1;
        }
        path.close();

        if num_points >= target {
            break;
        }
    }
    return path.build();
}

pub fn run(cmd: FuzzCmd) -> bool {
    let mut i: u64 = 0;
    println!("----");
    println!(
        "Fuzzing {} tessellation:",
        match (cmd.tess.fill, cmd.tess.stroke) {
            (Some(..), Some(..)) => "fill and stroke",
            (_, Some(..)) => "stroke",
            _ => "fill",
        }
    );
    if let Some(num) = cmd.min_points {
        println!("minimum number of points per path: {}", num);
    }
    if let Some(num) = cmd.max_points {
        println!("maximum number of points per path: {}", num);
    }
    println!("----");
    loop {
        let path = generate_path(&cmd, i);
        if let Some(options) = cmd.tess.fill {
            let status = ::std::panic::catch_unwind(|| {
                if cmd.rasterize {
                    rasterize(&path, &options)
                } else {
                    match cmd.tess.tessellator {
                        Tessellator::Default => {
                            let result = FillTessellator::new().tessellate(
                                &path,
                                &options,
                                &mut NoOutput::new(),
                            );
                            if !cmd.ignore_errors {
                                result.unwrap();
                            }
                        }
                        Tessellator::Tess2 => {
                            let result = tess2::FillTessellator::new().tessellate(
                                &path,
                                &options,
                                &mut NoOutput::new(),
                            );
                            if !cmd.ignore_errors {
                                result.unwrap();
                            }
                        }
                }
                }
            });

            if status.is_err() {
                println!(" !! Error while tessellating");
                println!("    Path #{}", i);
                find_reduced_test_case(path.as_slice(), &|path: Path| {
                    if cmd.rasterize {
                        rasterize(&path, &options);
                        false
                    } else {
                        FillTessellator::new()
                            .tessellate(&path, &options, &mut NoOutput::new())
                            .is_err()
                    }
                });

                panic!("aborting");
            }
        }
        if let Some(options) = cmd.tess.stroke {
            StrokeTessellator::new()
                .tessellate(&path, &options, &mut NoOutput::new())
                .unwrap();
        }

        if let Some(ref hatch) = cmd.tess.hatch {
            let mut builder = Path::builder();
            let mut hatcher = Hatcher::new();
            hatcher.hatch_path(
                path.iter(),
                &hatch.options,
                &mut RegularHatchingPattern {
                    interval: hatch.spacing,
                    callback: &mut |segment: &HatchSegment| {
                        builder.add_line_segment(&LineSegment {
                            from: segment.a.position,
                            to: segment.b.position,
                        });
                    },
                },
            );
            let _hatched_path = builder.build();
        }

        if let Some(ref dots) = cmd.tess.dots {
            let mut builder = Path::builder();
            let mut hatcher = Hatcher::new();
            hatcher.dot_path(
                path.iter(),
                &dots.options,
                &mut RegularDotPattern {
                    row_interval: dots.spacing,
                    column_interval: dots.spacing,
                    callback: &mut |dot: &Dot| {
                        builder.add_point(dot.position);
                    },
                },
            );
            let _dotted_path = builder.build();
        }

        i += 1;
        if i % 500 == 0 {
            println!(" -- tested {} paths", i);
        }
    }
}
