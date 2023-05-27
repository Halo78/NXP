
#ifndef NXPCUP_RACE_
#define NXPCUP_RACE_

#include <px4_defines.h>
#include <uORB/topics/pixy_vector.h>

#define SPEED_FAST	0.16f
#define SPEED_NORMAL 0.14f
#define SPEED_SLOW	0.10f
#define SPEED_STOP	0.0f

struct roverControl {
	float steer;
	float speed;
};

struct _vector {
	float x;
	float y;
	float norm;
	float grad;
};

struct Vector
{
	void print()
	{
		char buf[64];
		sprintf(buf, "vector: (%d %d) (%d %d)", m_x0, m_y0, m_x1, m_y1);
		printf(buf);
		printf("\n");
	}

	uint8_t m_x0;
	uint8_t m_y0;
	uint8_t m_x1;
	uint8_t m_y1;
};

roverControl raceTrack(const pixy_vector_s &pixy);
uint8_t get_num_vectors(Vector &vec1, Vector &vec2);
Vector copy_vectors(pixy_vector_s &pixy, uint8_t num);

#endif
