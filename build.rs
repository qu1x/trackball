fn main() {
	#[cfg(feature = "cc")]
	{
		use std::path::Path;
		let src = Path::new("c11").join("src");
		cc::Build::new()
			.files(&[src.join("trackball.c")])
			.include(src)
			.flag("-std=c11")
			.flag("-Wall")
			.flag("-Wextra")
			.flag("-Werror")
			.flag("-pedantic")
			.compile("trackball");
	}
}
