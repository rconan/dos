use cc;

fn main() {
    cc::Build::new()
        .file("src/controllers/mount/controller/MountControl0.c")
        .file("src/controllers/mount/controller/MountControl0_data.c")
        .compile("mount_controller");
    cc::Build::new()
        .file("src/controllers/mount/drives/MountDrives.c")
        .file("src/controllers/mount/drives/MountDrives_data.c")
        .compile("mount_drives");
    cc::Build::new()
        .file("src/controllers/m1/hp_load_cells/M1HPloadcells.c")
        .compile("m1_hp_load_cells");
    cc::Build::new()
        .file("src/controllers/m1/local_controller/M1LocalControl.c")
        .file("src/controllers/m1/local_controller/M1LocalControl_data.c")
        .compile("m1_local_controller");
}
