if [ ! -d MLX42 ]; then
    git clone https://github.com/codam-coding-college/MLX42.git
fi
cd MLX42
cmake -B build
cmake --build build
cd ..
gcc raymarcher.c MLX42/build/libmlx42.a -lpthread -O3 -ffast-math -flto -mavx2 -mfma -ftree-vectorize -lglfw -lm -o raymarcher
