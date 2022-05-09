add_rules("mode.debug", "mode.release")

target("pvs2d")
    set_kind("static")
    add_files("src/pvs2d.c")
    add_includedirs("include")
    add_includedirs("include", {public = true})
    if is_mode("debug") then
        add_defines("DEBUG")
    end
