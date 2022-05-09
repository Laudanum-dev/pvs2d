add_rules("mode.debug", "mode.release")
add_requires("raylib")

target("pvs2d")
    set_kind("static")
    add_files("src/pvs2d.c")
    add_includedirs("include")
    add_includedirs("include", {public = true})
    if is_mode("debug") then
        add_defines("DEBUG")
    end

target("demo")
    set_kind("binary")
    add_files("src/demo.c")
    add_deps("pvs2d")
    add_packages("raylib")
    after_build(function(target) 
        -- os.run("copy resources\\* %s", target:targetdir())
        -- os.execute("cd")
        -- io.popen("cd")
        os.run("afterbuild.bat %s", target:targetdir())
    end)
