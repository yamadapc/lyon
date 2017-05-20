use api::*;
use core::math::*;
use renderer::{GpuFillVertex, GpuStrokeVertex};
use path::Path;
use tessellation::geometry_builder::VertexBuffers;
use batch_builder::GeometryStore;
use gpu_data::{GpuRect};
use data_store::DataStore;

use std::sync::Arc;

type Scene = SceneDescriptor;

pub struct Context {
    buffers: DataStore,
    shapes: ShapeStore,
    fill_geom: GeometryStore<GpuFillVertex>,
    stroke_geom: GeometryStore<GpuFillVertex>,
    scenes: Vec<Scene>,
    render_targets: Vec<RenderTarget>,
    render_surfaces: Vec<RenderSurface>,
}

struct RenderTarget {
    descriptor: RenderTargetDescriptor,
    surface: Option<RenderSurfaceId>,
    scene: SceneId,
}

struct RenderSurface {
    format: SurfaceFormat,
    width: u16,
    height: u16,
}

impl Api for Context {
    fn add_path(&mut self, path: PathDescriptor) -> ShapeId { self.shapes.add_path(path) }

    fn add_colors(&mut self, colors: &[Color], usage: Usage) -> ColorIdRange {
        self.buffers.add_colors(colors, usage)
    }

    fn add_transforms(&mut self, transforms: &[Transform], usage: Usage) -> TransformIdRange {
        self.buffers.add_transforms(transforms, usage)
    }

    fn add_numbers(&mut self, values: &[f32], usage: Usage) -> NumberIdRange {
        self.buffers.add_numbers(values, usage)
    }

    fn add_points(&mut self, values: &[Point], usage: Usage) -> PointIdRange {
        self.buffers.add_points(values, usage)
    }

    fn add_rects(&mut self, values: &[GpuRect], usage: Usage) -> RectIdRange {
        self.buffers.add_rects(values, usage)
    }

    fn add_gradient_stops(&mut self, _gradient: &[GradientStop], _usage: Usage) -> GradientId {
        unimplemented!();
    }

    fn add_scene(&mut self, descriptor: &SceneDescriptor) -> SceneId {
        self.scenes.push(descriptor.clone());
        SceneId::from_index(self.scenes.len() - 1)
    }

    fn add_render_target(
        &mut self,
        descriptor: &RenderTargetDescriptor,
        scene: SceneId,
    ) -> RenderTargetId {
        self.render_targets
            .push(
                RenderTarget {
                    descriptor: descriptor.clone(),
                    surface: None,
                    scene: scene,
                }
            );
        RenderTargetId::from_index(self.render_targets.len() - 1)
    }


    fn set_colors(&mut self, range: ColorIdRange, values: &[Color]) {
        self.buffers.set_colors(range, values);
    }

    fn set_transforms(&mut self, range: TransformIdRange, values: &[Transform]) {
        self.buffers.set_transforms(range, values);
    }

    fn set_numbers(&mut self, range: NumberIdRange, values: &[f32]) {
        self.buffers.set_numbers(range, values);
    }

    fn set_points(&mut self, range: PointIdRange, values: &[Point]) {
        self.buffers.set_points(range, values);
    }

    fn set_rects(&mut self, range: RectIdRange, values: &[GpuRect]) {
        self.buffers.set_rects(range, values);
    }

    fn set_gradient_stops(&mut self, id: GradientId, stops: &[GradientStop]) {
        unimplemented!();
    }

    fn remove_shape(&mut self, shape: ShapeId) {
        unimplemented!();
    }

    fn remove_scene(&mut self, scene: SceneId) {
        unimplemented!();
    }


    fn render(&mut self, device: &mut Device) -> Result<(), ()> {
        unimplemented!();
    }
}

impl Context {
    fn compute_prim_count(&self, scene: SceneId) -> usize {
        let scene = &self.scenes[scene.index()];
        let mut count = 0;
        for item in &scene.items {
            match item {
                &RenderItem::Paint(_) => {
                    count += 1;
                }
                &RenderItem::Scene(ref instance) => {
                    count += self.compute_prim_count(instance.id);
                }
            }
        }
        return count;
    }
}

pub struct ShapeStore {
    paths: Vec<PathDescriptor>,
}

impl ShapeStore {
    pub fn new() -> Self { Self { paths: Vec::new() } }

    pub fn add_path(&mut self, descriptor: PathDescriptor) -> ShapeId {
        self.paths.push(descriptor);
        ShapeId::Path(PathId::from_index(self.paths.len() - 1))
    }

    pub fn get_path(&self, id: PathId) -> &Arc<Path> { &self.paths[id.index()].path }
}
