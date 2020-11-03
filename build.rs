use std::env;
use std::path::Path;

fn main() {
    #[cfg(not(feature = "workaround_rls_vscode_bug_586"))]
    {
        let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let profile = env::var("PROFILE").unwrap();
        let target_path = Path::new(&crate_dir)
            .join("target")
            .join(profile)
            .join("leeward.h");
        cbindgen::Builder::new()
            .with_crate(crate_dir)
            .generate()
            .expect("Unable to generate bindings")
            .write_to_file(target_path);
    }
}
