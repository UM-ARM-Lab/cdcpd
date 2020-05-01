# 
using LinearAlgebra
using DelimitedFiles

# P_py = readdlm("np_P_0.txt")
# P_cpp = readdlm("cpp_P_0.txt")

# TY_py = readdlm("np_TY_0.txt")
# TY_cpp = readdlm("cpp_TY_0.txt")

for file in [
             "Y.txt",
             "TY.txt",
             "Y_opt.txt",
            ]
    x_py = readdlm("np_" * file)
    x_cpp = readdlm("cpp_" * file)
    print(size(x_py))
    print(size(x_cpp))
    print(file)
    print(" ")
    println(maximum(abs.(x_py - x_cpp)))
end
