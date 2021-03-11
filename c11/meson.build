project('trackball', 'c',
	version : '1.0.0',
	license : 'BSD-3-Clause',
	default_options : ['c_std=c11'])

c = meson.get_compiler('c')
m = c.find_library('m',
	required : true)

src = 'src'

install_headers(join_paths(src, meson.project_name() + '.h'))
trackball = library(meson.project_name(),
	join_paths(src, meson.project_name() + '.c'),
	dependencies : m,
	c_args : ['-Wall', '-Wextra', '-Werror', '-pedantic'],
	install : true)

pkg = import('pkgconfig')
pkg.generate(libraries : trackball,
	name : meson.project_name(),
	description : 'Virtual Trackball Orbiting via the Exponential Map')

example = executable('example',
	join_paths('examples', 'example.c'),
	include_directories: src,
	link_with: trackball)
test('Example', example)
