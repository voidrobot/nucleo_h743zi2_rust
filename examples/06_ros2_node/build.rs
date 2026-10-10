use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn collect_c_files(dir: &Path, files: &mut Vec<PathBuf>, excluded_dirs: &[&str]) {
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                let dir_name = path.file_name().unwrap_or_default().to_string_lossy();
                if !excluded_dirs.iter().any(|&ex| dir_name == ex) {
                    collect_c_files(&path, files, excluded_dirs);
                }
            } else if path.extension().map_or(false, |ext| ext == "c") {
                files.push(path);
            }
        }
    }
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let repos_dir = manifest_dir.join("../../repos");
    let picoros_dir = repos_dir.join("Pico-ROS-software");
    let zenoh_dir = picoros_dir.join("thirdparty/zenoh-pico");
    let ucdr_dir = picoros_dir.join("thirdparty/Micro-CDR");

    let debug_log = manifest_dir.join("build_debug.log");
    let _ = fs::write(&debug_log, "build.rs started\n");

    let mut build = cc::Build::new();

    // arm-none-eabi-gcc 컴파일러 설정
    build.compiler("arm-none-eabi-gcc");
    build.target("thumbv7em-none-eabihf");
    build.flag("-mcpu=cortex-m7");
    build.flag("-mfpu=fpv5-d16");
    build.flag("-mfloat-abi=hard");
    build.flag("-mthumb");
    build.flag("-specs=nano.specs");

    // 1. C 표준 및 컴파일러 옵션
    build.std("c11");

    // 2. 전처리기 정의
    build.define("ZENOH_C_STANDARD", "11");
    build.define("ZENOH_GENERIC", "1");
    build.define("PICOROS_DEBUG", None);
    build.define("USER_TYPE_FILE", "\"example_types.h\"");

    // 3. 헤더 인클루드 경로 설정
    build.include(manifest_dir.join("c_compat/include"));
    build.include(picoros_dir.join("src"));
    build.include(picoros_dir.join("examples"));
    build.include(picoros_dir.join("thirdparty/config"));
    build.include(ucdr_dir.join("include"));
    build.include(zenoh_dir.join("include"));
    build.include(zenoh_dir.join("include/zenoh-pico"));
    build.include(zenoh_dir.join("src"));

    // 4. 소스 파일 수집
    let mut c_files = Vec::new();

    // (0) Embassy 베어메탈 어댑터 (UDP & System)
    c_files.push(manifest_dir.join("c_compat/src/udp_embassy.c"));
    c_files.push(manifest_dir.join("c_compat/src/system_embassy.c"));
    c_files.push(manifest_dir.join("c_compat/src/picoros_wrapper.c"));

    // (1) Pico-ROS 코어
    c_files.push(picoros_dir.join("src/picoros.c"));
    c_files.push(picoros_dir.join("src/picoserdes.c"));
    c_files.push(picoros_dir.join("src/picoparams.c"));

    // (2) Micro-CDR
    collect_c_files(&ucdr_dir.join("src/c"), &mut c_files, &[]);

    // (3) zenoh-pico 코어 (MT/OS 종속 파일 배제)
    let zenoh_src = zenoh_dir.join("src");
    collect_c_files(&zenoh_src.join("api"), &mut c_files, &[]);
    collect_c_files(&zenoh_src.join("collections"), &mut c_files, &["fifo_mt.c", "ring_mt.c"]);
    collect_c_files(&zenoh_src.join("net"), &mut c_files, &[]);
    collect_c_files(&zenoh_src.join("protocol"), &mut c_files, &["serial.c"]);
    collect_c_files(&zenoh_src.join("runtime"), &mut c_files, &[]);
    collect_c_files(&zenoh_src.join("session"), &mut c_files, &[]);
    collect_c_files(&zenoh_src.join("transport/common"), &mut c_files, &[]);
    collect_c_files(&zenoh_src.join("transport/unicast"), &mut c_files, &[]);
    c_files.push(zenoh_src.join("transport/transport.c"));
    c_files.push(zenoh_src.join("transport/manager.c"));
    c_files.push(zenoh_src.join("transport/unicast.c"));
    c_files.push(zenoh_src.join("transport/peer.c"));
    c_files.push(zenoh_src.join("transport/utils.c"));
    collect_c_files(&zenoh_src.join("utils"), &mut c_files, &[]);

    // Link transport (공통 및 UDP 엔드포인트)
    c_files.push(zenoh_src.join("link/endpoint.c"));
    c_files.push(zenoh_src.join("link/link.c"));
    c_files.push(zenoh_src.join("link/config/udp.c"));
    c_files.push(zenoh_src.join("link/transport/common/endpoints.c"));
    c_files.push(zenoh_src.join("link/transport/common/address.c"));
    c_files.push(zenoh_src.join("link/transport/udp/address.c"));
    c_files.push(zenoh_src.join("link/unicast/udp.c"));

    let mut log_content = format!("Collected {} C files:\n", c_files.len());
    for file in &c_files {
        log_content.push_str(&format!("  {}\n", file.display()));
        build.file(file);
    }
    let _ = fs::write(&debug_log, &log_content);

    // 5. 정적 라이브러리 빌드 및 링크
    build.compile("picoros");

    let _ = fs::write(&debug_log, "build.compile finished successfully\n");

    // ARM Cortex-M7 newlib-nano 및 libgcc C 표준 라이브러리 링크
    println!("cargo:rustc-link-search=native=/usr/lib/arm-none-eabi/newlib/thumb/v7e-m+dp/hard");
    println!("cargo:rustc-link-search=native=/usr/lib/gcc/arm-none-eabi/13.2.1/thumb/v7e-m+dp/hard");
    println!("cargo:rustc-link-lib=c_nano");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=nosys");
    println!("cargo:rustc-link-lib=gcc");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=c_compat");
    println!("cargo:rerun-if-changed={}", picoros_dir.display());
}
