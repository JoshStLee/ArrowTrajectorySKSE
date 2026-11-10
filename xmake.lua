set_xmakever('3.0.1')

-- Disable Skyrim VR by default since this is an SE/AE plugin template
option('skyrim_vr', function()
    set_default(false)
    set_showmenu(true)
    set_description('Enable runtime support for Skyrim VR')
end)

includes('lib/commonlibsse-ng')

set_project('ArrowTrajectory')
set_version('1.0.0')
set_license('GPL-3.0')

set_languages('c++23')
set_warnings('allextra')
set_policy('package.requires_lock', true)
set_toolset('msvc', 'ninja')

add_rules('mode.debug', 'mode.releasedbg', 'mode.release')

target('ArrowTrajectory')
    set_kind('shared')
    add_deps('commonlibsse-ng')

    add_rules('commonlibsse-ng.plugin', {
        name        = 'ArrowTrajectory',
        author      = 'LeStLee',
        description = 'Standalone ballistic trajectory and raycast crosshair plugin'
    })

    add_files('src/**.cpp')
    add_headerfiles('src/**.h')

    add_includedirs(
        'src',
        '$(projectdir)'
    )

    set_pcxxheader('src/pch.h')
