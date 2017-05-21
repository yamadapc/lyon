use core::math::{Transform2D, Mat4};
use std::mem;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct GpuBlock4([f32; 4]);

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct GpuBlock8([f32; 8]);

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct GpuBlock16([f32; 16]);

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct GpuBlock32([f32; 32]);

macro_rules! default_uninitialized {
    ($Type:ident) => (
        impl Default for $Type {
            fn default() -> Self {
                $Type(unsafe { mem::uninitialized() })
            }
        }
    )
}

default_uninitialized!(GpuBlock4);
default_uninitialized!(GpuBlock8);
default_uninitialized!(GpuBlock16);
default_uninitialized!(GpuBlock32);

macro_rules! gpu_data_impl {
    (
        struct $Type:ident : $BlockType:ident {
            $($field_name:ident : $FieldType:ty,)*
        }
    ) => (
        impl Copy for $Type {}
        impl Clone for $Type { fn clone(&self) -> Self { *self } }

        impl $Type {
            pub fn new($($field_name: $FieldType,)*) -> Self {
                $Type {
                    $($field_name: $field_name,)*
                    .. unsafe { mem::uninitialized() }
                }
            }
        }

        impl Into<$BlockType> for $Type {
            fn into(self) -> $BlockType {
                unsafe { mem::transmute(self) }
            }
        }

        impl From<$BlockType> for $Type {
            fn from(block: $BlockType) -> $Type {
                unsafe { mem::transmute(block) }
            }
        }
    )
}

#[macro_export]
macro_rules! gpu_data {
    (
        #[padding($padding:expr)]
        struct $Type:ident : $BlockType:ident {
            $($field_name:ident : $FieldType:ty,)*
        }
    ) => (
        #[repr(C)]
        #[derive(Debug)]
        pub struct $Type {
            $(pub $field_name: $FieldType,)*
            _padding: [u32; $padding],
        }

        gpu_data_impl! {
            struct $Type : $BlockType {
                $($field_name: $FieldType,)*
            }
        }
    );

    (
        struct $Type:ident : $BlockType:ident {
            $($field_name:ident : $FieldType:ty,)*
        }
    ) => (
        #[repr(C)]
        #[derive(Debug)]
        pub struct $Type {
            $(pub $field_name: $FieldType,)*
        }

        gpu_data_impl! {
            struct $Type : $BlockType {
                $($field_name: $FieldType,)*
            }
        }
    )

}

gpu_data! {
    struct GpuColorF: GpuBlock4 {
        r: f32,
        g: f32,
        b: f32,
        a: f32,
    }
}

gpu_data! {
    struct GpuRect: GpuBlock4 {
        top_left: [f32; 2],
        bottom_right: [f32; 2],
    }
}

gpu_data! {
    #[padding(2)]
    struct GpuTransform2D: GpuBlock8 {
        matrix: Transform2D,
    }
}

gpu_data! {
    struct GpuTransform3D: GpuBlock16 {
        matrix: Mat4,
    }
}

gpu_data! {
    #[padding(1)]
    struct FillPrimitive: GpuBlock8 {
        texture_rect: GpuRect,
        //pub color: [f32; 4],
        z_index: f32,
        local_transform: i32,
        view_transform: i32,
    }
}
