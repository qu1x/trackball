fn main() {
	#[cfg(feature = "cc")]
	{
		use std::path::Path;
		let src = Path::new("c11").join("src");
		cc::Build::new()
			.files(&[src.join("trackball.c")])
			.include(src)
			.flag_if_supported("-lm")
			.flag_if_supported("-pedantic")
			.flag_if_supported("-std=c11")
			.warnings_into_errors(true)
			.compile("trackball");
	}
}
