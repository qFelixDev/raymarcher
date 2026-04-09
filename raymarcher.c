#define INITIAL_WIDTH 720
#define INITIAL_HEIGHT 720
#define EPSILON 0.00001
#define THREAD_COUNT 24
#include<stdint.h>
#include<stdio.h>
#include<math.h>
#include<float.h>
#include<pthread.h>
#include<stdatomic.h>
#include"MLX42/include/MLX42/MLX42.h"

typedef struct {
	float	x;
	float	y;
} float_v2_t;

typedef struct {
	float	x;
	float	y;
	float	z;
} float_v3_t;

typedef struct {
	uint32_t	x;
	uint32_t	y;
} uint32_v2_t;

static struct {
	mlx_t*				instance;
	mlx_image_t*		framebuffer;
	float_v3_t			position;
	float_v3_t			movement;
	float				speed;
	float				drawDistance;
	float				pitch;
	float				yaw;
	bool				isCursorBound;
	pthread_barrier_t	renderStart;
	pthread_barrier_t	renderEnd;
	_Atomic uint32_t	currentScanline;
	float_v2_t			viewBound;
	float_v2_t			viewIncrement;
	bool				isRunning;
} raymarcher;

float_v3_t normalize(float_v3_t vector) {
	float hypotenuse = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	return (float_v3_t) {
		vector.x / hypotenuse,
		vector.y / hypotenuse,
		vector.z / hypotenuse
	};
}

float scalar(float_v3_t vectorA, float_v3_t vectorB) {
	return vectorA.x * vectorB.x + vectorA.y * vectorB.y + vectorA.z * vectorB.z;
}

float sphere(float_v3_t position) {
	return sqrt(position.x * position.x + position.y * position.y + position.z * position.z) - 1;
}

float cube(float_v3_t position, float radius) {
	position.x = fabs(position.x);
	position.y = fabs(position.y);
	position.z = fabs(position.z);
	position.x -= radius;
	position.y -= radius;
	position.z -= radius;
	if(position.x > 0 || position.y > 0 || position.z > 0) {
		float squaredDistance = 0;
		squaredDistance += position.x * position.x * (position.x > 0);
		squaredDistance += position.y * position.y * (position.y > 0);
		squaredDistance += position.z * position.z * (position.z > 0);
		return sqrt(squaredDistance);
	} else { // There is only one quadrant of the absolute cube cutout that is actually inside of the cube and that is the --- quadrant
		return fmax(position.x, fmax(position.y, position.z));
	}
}

/*float mengerSponge(float_v3_t position) {
float distance = cube(position, 1.5 - EPSILON);
distance = fmax(distance, -cube(position, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x + 1.0, position.y, position.z}, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x - 1.0, position.y, position.z}, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x, position.y + 1.0, position.z}, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x, position.y - 1.0, position.z}, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x, position.y, position.z + 1.0}, 0.5 + EPSILON));
distance = fmax(distance, -cube((float_v3_t) {position.x, position.y, position.z - 1.0}, 0.5 + EPSILON));
return distance;
}*/

float cubeSDF(float_v3_t rayPosition, float_v3_t cubePosition, float length) {
	rayPosition.x -= cubePosition.x;
	rayPosition.y -= cubePosition.y;
	rayPosition.z -= cubePosition.z;
	float radius = length / 2 + EPSILON;
	rayPosition.x = fabs(rayPosition.x - radius) - radius;
	rayPosition.y = fabs(rayPosition.y - radius) - radius;
	rayPosition.z = fabs(rayPosition.z - radius) - radius;
	if(rayPosition.x > 0 || rayPosition.y > 0 || rayPosition.z > 0) {
		float squaredDistance = 0;
		squaredDistance += rayPosition.x * rayPosition.x * (rayPosition.x > 0);
		squaredDistance += rayPosition.y * rayPosition.y * (rayPosition.y > 0);
		squaredDistance += rayPosition.z * rayPosition.z * (rayPosition.z > 0);
		return sqrt(squaredDistance);
	} else {
		return fmax(rayPosition.x, fmax(rayPosition.y, rayPosition.z));
	}
}

float cutCube(float distance, float_v3_t rayPosition, float_v3_t cubePosition, float length) {
	float subLength = length / 3;
	float_v3_t center = {
		cubePosition.x + subLength,
		cubePosition.y + subLength,
		cubePosition.z + subLength
	};
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x, center.y, center.z}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x - subLength, center.y, center.z}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x + subLength, center.y, center.z}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x, center.y - subLength, center.z}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x, center.y + subLength, center.z}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x, center.y, center.z - subLength}, subLength + 2 * EPSILON));
	distance = fmax(distance, -cubeSDF(rayPosition, (float_v3_t) {center.x, center.y, center.z + subLength}, subLength + 2 * EPSILON));
	return distance;
}

float mengerSponge(float_v3_t rayPosition, float_v3_t spongePosition, float length) {
	rayPosition.x -= spongePosition.x;
	rayPosition.y -= spongePosition.y;
	rayPosition.z -= spongePosition.z;
	float distance = cubeSDF(rayPosition, (float_v3_t) {2 * EPSILON, 2 * EPSILON, 2 * EPSILON}, 1.5 - 4 * EPSILON);
	distance = cutCube(distance, rayPosition, (float_v3_t) {0, 0, 0}, 1.5);
	return distance;
}

#define MAX_ITERATIONS 5

float_v3_t determineClosestSubarea(float_v3_t subarea, float_v3_t position, float length) {
	float radius = length / 2;
	position.x -= subarea.x - radius;
	position.y -= subarea.y - radius;
	position.z -= subarea.z - radius;
	float_v3_t absolute = {
		fabs(position.x),
		fabs(position.y),
		fabs(position.z)
	};
	if(absolute.x > absolute.y) {
		if(absolute.x < absolute.z) {
			subarea.x += length * (1 - 2 * (position.x < 0));
			return subarea;
		}
	} else {
		if(absolute.y > absolute.z) {
			subarea.y += length * (1 - 2 * (position.y < 0));
			return subarea;
		}
	}
	subarea.z += length * (1 - 2 * (position.z < 0));
	return subarea;
}

float mengerSpongeSDF(float_v3_t rayPosition, float_v3_t spongePosition, float length) {
	rayPosition.x -= spongePosition.x;
	rayPosition.y -= spongePosition.y;
	rayPosition.z -= spongePosition.z;
	float distance = cubeSDF(rayPosition, (float_v3_t) {2 * EPSILON, 2 * EPSILON, 2 * EPSILON}, length - 4 * EPSILON);
	distance = cutCube(distance, rayPosition, (float_v3_t) {0, 0, 0}, length);
	int iteration = 0;
	while(iteration < MAX_ITERATIONS && distance < length) {
		length /= 3 + EPSILON;
		float_v3_t subarea = {
			floor(rayPosition.x / length) * length,
			floor(rayPosition.y / length) * length,
			floor(rayPosition.z / length) * length
		};
		if(distance > EPSILON) {
			subarea = determineClosestSubarea(subarea, rayPosition, length);
		}
		distance = cutCube(distance, rayPosition, subarea, length);
		iteration++;
	}
	return distance;
}

float ufmod(float x, float y) {
	return fmod(x, y) + (x < 0) * y;
}

/*float getDistance(float_v3_t position) {
	position.x = fabs(ufmod(position.x, 10.0));
	position.y = fabs(ufmod(position.y, 10.0));
	position.z = fabs(ufmod(position.z, 10.0));
	// NOTE: The object that the modulus operator is applied to must be centered or there will be artifacts due to overstepping
	return fmax(
		cube((float_v3_t) {position.x - 5.0, position.y - 5.0, position.z - 5.0}, 0.5),
		-cube((float_v3_t) {position.x - 5.0, position.y - 5.0, position.z - 4.5}, 0.4)
	);
	//return sphere((float_v3_t) {position.x, position.y, position.z - 5});
}*/
float getDistance(float_v3_t position) {
	return mengerSpongeSDF(position, (float_v3_t) {0.1, 0.1, 0.1}, 1.5);
}

float_v3_t getNormal(float_v3_t position) {
	return normalize((float_v3_t) {
		+ getDistance((float_v3_t) {position.x + EPSILON, position.y, position.z})
		- getDistance((float_v3_t) {position.x - EPSILON, position.y, position.z}),
		+ getDistance((float_v3_t) {position.x, position.y + EPSILON, position.z})
		- getDistance((float_v3_t) {position.x, position.y - EPSILON, position.z}),
		+ getDistance((float_v3_t) {position.x, position.y, position.z + EPSILON})
		- getDistance((float_v3_t) {position.x, position.y, position.z - EPSILON})
	});
}

float_v3_t raymarch(float_v3_t origin, float_v3_t direction) {
	float distance = FLT_MAX;
	float traveled = 0;
	float_v3_t position = origin;
	uint32_t steps = 0;
	while(distance > EPSILON && traveled < raymarcher.drawDistance) {
		distance = getDistance(position);
		if(distance < 0)
			distance = 0;
		position.x += direction.x * distance;
		position.y += direction.y * distance;
		position.z += direction.z * distance;
		traveled += distance;
		steps++;
	}
	/*float brightness = 1 - (traveled / 100);
	if((int)(floor(position.x * 4) + floor(position.y * 4) + floor(position.z * 4)) % 2 == 0)
		return ((float_v3_t) {brightness, 0, 0});
	else
		return ((float_v3_t) {0, 0, brightness});*/
	float currentTime = mlx_get_time();
	float brightness = (scalar(getNormal(position), (float_v3_t) {cos(currentTime), sin(currentTime), cos(currentTime * 0.5)}) + 1.5) * 0.2
		* (1 - (traveled / raymarcher.drawDistance));
	if(distance > EPSILON) {
		return (float_v3_t) {0, 0, 0};
	} else {
		return (float_v3_t) {brightness, brightness, brightness};
	}
}

float clamp(float min, float val, float max) {
	if(val < min)
		return min;
	else if(val > max)
		return max;
	else
		return val;
}

float_v3_t rotateX(float_v3_t vector, float yaw) {
	 return (float_v3_t) {
		 vector.x,
		 vector.y * cos(yaw) + vector.z * sin(yaw),
		 vector.y * -sin(yaw) + vector.z * cos(yaw)
	 };
}

float_v3_t rotateY(float_v3_t vector, float pitch) {
	return (float_v3_t) {
		vector.x * cos(pitch) + vector.z * sin(pitch),
		vector.y,
		vector.x * -sin(pitch) + vector.z * cos(pitch)
	};
}

void* renderThread(void* param) {
	while(true) {
		pthread_barrier_wait(&raymarcher.renderStart);
		if(!raymarcher.isRunning)
			return NULL;
		uint32_v2_t imagePosition = {0, atomic_fetch_add(&raymarcher.currentScanline, 1)};
		float_v2_t	viewPosition = {-raymarcher.viewBound.x, raymarcher.viewBound.y - imagePosition.y * raymarcher.viewIncrement.y};
		while(imagePosition.y < raymarcher.framebuffer -> height) {
			float_v3_t direction = normalize((float_v3_t) {viewPosition.x, viewPosition.y, 1});
			float_v3_t color = raymarch(raymarcher.position, rotateY(rotateX(direction, raymarcher.yaw), raymarcher.pitch));
			// NOTE: Remember to ALWAYS multiply scalar image positions by 4
			uint8_t* pixels = raymarcher.framebuffer -> pixels + 4 * (imagePosition.x + imagePosition.y * raymarcher.framebuffer -> width);
			*(pixels++) = (uint8_t)(clamp(0, color.x, 0.99) * 256);
			*(pixels++) = (uint8_t)(clamp(0, color.y, 0.99) * 256);
			*(pixels++) = (uint8_t)(clamp(0, color.z, 0.99) * 256);
			*(pixels++) = 255;
			imagePosition.x++;
			viewPosition.x += raymarcher.viewIncrement.x;
			if(imagePosition.x < raymarcher.framebuffer -> width)
				continue;
			imagePosition.x = 0;
			imagePosition.y = atomic_fetch_add(&raymarcher.currentScanline, 1);
			viewPosition.x = -raymarcher.viewBound.x;
			viewPosition.y = raymarcher.viewBound.y - imagePosition.y * raymarcher.viewIncrement.y;
		}
		pthread_barrier_wait(&raymarcher.renderEnd);
	}
}

void onLoop(void* param) {
	float frameStart = mlx_get_time();

	raymarcher.viewBound = (float_v2_t) {(float)raymarcher.framebuffer -> width / (float)raymarcher.framebuffer -> height, 1};
	raymarcher.viewIncrement.x = 2 * (raymarcher.viewBound.x / (float)raymarcher.framebuffer -> width);
	raymarcher.viewIncrement.y = 2 * (raymarcher.viewBound.y / (float)raymarcher.framebuffer -> height);

	raymarcher.currentScanline = 0;
	pthread_barrier_wait(&raymarcher.renderStart);
	pthread_barrier_wait(&raymarcher.renderEnd);
	float frameTime = mlx_get_time() - frameStart;
	float_v3_t movement = rotateY(raymarcher.movement, raymarcher.pitch);
	raymarcher.position.x += movement.x * raymarcher.speed * frameTime;
	raymarcher.position.y += movement.y * raymarcher.speed * frameTime;
	raymarcher.position.z += movement.z * raymarcher.speed * frameTime;
	printf("%f fps\n", 1 / frameTime);
	printf("%f %f %f\n", raymarcher.position.x, raymarcher.position.y, raymarcher.position.z);
}

void onResize(int32_t width, int32_t height, void* param) {
	mlx_resize_image(raymarcher.framebuffer, width, height);
}

void onKeyInput(mlx_key_data_t keydata, void* param) {
	if(keydata.action == MLX_PRESS) {
		puts("press");
		switch(keydata.key) {
			case MLX_KEY_W:
				raymarcher.movement.z = 1;
				break;
			case MLX_KEY_A:
				raymarcher.movement.x = -1;
				break;
			case MLX_KEY_S:
				raymarcher.movement.z = -1;
				break;
			case MLX_KEY_D:
				raymarcher.movement.x = 1;
				break;
			case MLX_KEY_SPACE:
				raymarcher.movement.y = 1;
				break;
			case MLX_KEY_LEFT_SHIFT:
				raymarcher.movement.y = -1;
				break;
			case MLX_KEY_B:
				raymarcher.isCursorBound = !raymarcher.isCursorBound;
				break;
			case MLX_KEY_UP:
				raymarcher.speed *= 2;
				break;
			case MLX_KEY_DOWN:
				raymarcher.speed /= 2;
				break;
		}
	} else if(keydata.action == MLX_RELEASE) {
		switch(keydata.key) {
			case MLX_KEY_W:
				if(raymarcher.movement.z > 0)
					raymarcher.movement.z = 0;
				break;
			case MLX_KEY_A:
				if(raymarcher.movement.x < 0)
					raymarcher.movement.x = 0;
				break;
			case MLX_KEY_S:
				if(raymarcher.movement.z < 0)
					raymarcher.movement.z = 0;
				break;
			case MLX_KEY_D:
				if(raymarcher.movement.x > 0)
					raymarcher.movement.x = 0;
				break;
			case MLX_KEY_SPACE:
				if(raymarcher.movement.y > 0)
					raymarcher.movement.y = 0;
				break;
			case MLX_KEY_LEFT_SHIFT:
				if(raymarcher.movement.y < 0)
					raymarcher.movement.y = 0;
				break;
		}
	}
}

void onCursorMoved(double xpos, double ypos, void* param) {
	if(!raymarcher.isCursorBound)
		return;
	float cursorX = (xpos / (double)raymarcher.framebuffer -> width) * 2 - 1;
	float cursorY = (ypos / (double)raymarcher.framebuffer -> height) * 2 - 1;
	if(cursorX != 0 || cursorY != 0)
		mlx_set_mouse_pos(raymarcher.instance, raymarcher.framebuffer -> width / 2, raymarcher.framebuffer -> height / 2);
	raymarcher.pitch += cursorX * 0.1;
	raymarcher.yaw -= clamp(-1.57, cursorY * 0.1, 1.57);
}

void onScroll(double xdelta, double ydelta, void* param) {
	if(ydelta < 0)
		raymarcher.drawDistance /= 1.1;
	else
		raymarcher.drawDistance *= 1.1;
}

int main() {
	raymarcher.isRunning = true;
	raymarcher.speed = 1;
	raymarcher.drawDistance = 128;
	pthread_t threads[THREAD_COUNT];
	pthread_barrier_init(&raymarcher.renderStart, NULL, THREAD_COUNT + 1);
	pthread_barrier_init(&raymarcher.renderEnd, NULL, THREAD_COUNT + 1);
	int threadIndex = 0;
	while(threadIndex < THREAD_COUNT) {
		pthread_create(threads + threadIndex, NULL, renderThread, NULL);
		threadIndex++;
	}
	raymarcher.instance = mlx_init(INITIAL_WIDTH, INITIAL_HEIGHT, "Raymarcher", true);
	if(raymarcher.instance == NULL)
		return 1;
	raymarcher.framebuffer = mlx_new_image(raymarcher.instance, INITIAL_WIDTH, INITIAL_HEIGHT);
	mlx_image_to_window(raymarcher.instance, raymarcher.framebuffer, 0, 0);
	mlx_loop_hook(raymarcher.instance, onLoop, NULL);
	mlx_resize_hook(raymarcher.instance, onResize, NULL);
	mlx_key_hook(raymarcher.instance, onKeyInput, NULL);
	mlx_cursor_hook(raymarcher.instance, onCursorMoved, NULL);
	mlx_scroll_hook(raymarcher.instance, onScroll, NULL);
	mlx_loop(raymarcher.instance);
	raymarcher.isRunning = false;
	pthread_barrier_wait(&raymarcher.renderStart);
	threadIndex = 0;
	while(threadIndex < THREAD_COUNT) {
		pthread_join(threads[threadIndex], NULL);
		threadIndex++;
	}
	pthread_barrier_destroy(&raymarcher.renderStart);
	pthread_barrier_destroy(&raymarcher.renderEnd);
	mlx_terminate(raymarcher.instance);
}
