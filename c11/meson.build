project('trackball', 'c',
	version : '0.2.2',
	license : 'BSD-3-Clause',
	default_options : ['c_std=c11'])

name = meson.project_name()
root = 'src'

c = meson.get_compiler('c')
m = c.find_library('m',
	required : true)

install_headers(join_paths(root, name + '.h'))
trackball = library(name, join_paths(root, name + '.c'),
	dependencies : m,
	c_args : ['-Wall', '-Wextra', '-Werror', '-pedantic'],
	install : true)

pkg = import('pkgconfig')
pkg.generate(
	libraries : trackball,
	name : name,
	description : 'Virtual Trackball Orbiting via the Exponential Map')

example = executable('example', join_paths('examples', 'example.c'),
	include_directories : root,
	link_with : trackball)
test('Example', example)
