//
// Created by fisch on 24.11.2024.
//

#include "NUM_PARTICLES.h"
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <windows.h>
#include <math.h>

#define NUM_PARTICLES 1
#define NUM_RIGID_BODIES 1


#include <cstdio>

float random_uniform(int upper_bound)
{
    return rand() % upper_bound;
}


//Two-dimensional vector.

//Combination aus Struct(Wie Klasse) und typdef
typedef struct
{
    float x;
    float y;
} Vector2;

// Two dimensional particle
typedef struct
{
    Vector2 position;
    Vector2 velocity;
    float mass;
} Particle;

// Global array of particles.
Particle particles[NUM_PARTICLES];


//Prints all particles position to the ouput. We could instead draw them on screen
// in a more interesting application
void PrintParticles()
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle *particle = &particles[i];
        printf("particle[%i] (%.2f, %.2f)\n", i, particle->position.x, particle->position.y);
    }
}

//Initialize all particles with random position, zero velocities and 1kg mass.
void InitializeParticles()
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i].position = (Vector2) {random_uniform(50), random_uniform(50)};
        particles[i].velocity = (Vector2) {0, 0};
        particles[i].mass = 1;
    }
}

//Just applies Earth's gravity force (mass times gravity acceleration 9,81 m/s^2 to each particle
Vector2 ComputeForce(Particle *particle)
{
    return (Vector2) {0, particle->mass * -9.81f};
}

void RunSimulation()
{
    float totalSimulationTime = 10; //The Simulation will run for 10 seconds.
    float currentTime = 0; //This accumualates the time that has passed.
    float dt = 1; //Each step will take on second

    InitializeParticles();
    PrintParticles();

    while (currentTime < totalSimulationTime)
    {
        sleep(dt);

        for (int i = 0; i < NUM_PARTICLES; i++)
        {
            Particle *particle = &particles[i];
            Vector2 force = ComputeForce(particle);
            Vector2 acceleration = (Vector2) {force.x / particle->mass, force.y / particle->mass};
            particle->velocity.x += acceleration.x * dt;
            particle->velocity.y += acceleration.y * dt;
            particle->position.x += particle->velocity.x * dt;
            particle->position.y += particle->velocity.y * dt;
        }

        PrintParticles();
        currentTime += dt;
    }
}


typedef struct {
    float width;
    float height;
    float mass;
    float momentOfInertia;
} BoxShape;

void CalculateBoxInertia(BoxShape *boxShape) {
    float m = boxShape->mass;
    float w = boxShape->width;
    float h = boxShape->height;
    boxShape->momentOfInertia = m * (w * w + h * h) / 12;

}

typedef struct
{
    Vector2 position;
    Vector2 linearVelocity;
    float angle;
    float angularVelocity;
    Vector2 force;
    float torque;
    BoxShape shape;
} RigidBody;

RigidBody rigidBodies[NUM_RIGID_BODIES];

void PrintRigidBodies()
{
    for (int i = 0; i < NUM_RIGID_BODIES; i++)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        printf("body[%i] p = (%.2f, %.2f, a = %.2f\n", i, rigidBody->position.x, rigidBody->position.y, rigidBody->angle);
    }
}

void InitializeRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody *rigidBody = &rigidBodies[i];
        rigidBody->position = (Vector2){random_uniform(50), random_uniform(50)};
        rigidBody->angle = random_uniform(360)/360.f * M_PI * 2;
        rigidBody->linearVelocity = (Vector2){0, 0};
        rigidBody->angularVelocity = 0;

        BoxShape shape;
        shape.mass = 10;
        shape.width = 1 + random_uniform(2);
        shape.height = 1 + random_uniform(2);
        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;
    }
}

// Applies a force at a point in the body, inducing some torque.
void ComputeForceAndTorque(RigidBody *rigidBody) {
    Vector2 f = (Vector2){0, 100};
    rigidBody->force = f;
    // r is the 'arm vector' that goes from the center of mass to the point of force application
    Vector2 r = (Vector2){rigidBody->shape.width / 2, rigidBody->shape.height / 2};
    rigidBody->torque = r.x * f.y - r.y * f.x;
}


void RunRigidBodySimulation() {
    float totalSimulationTime = 10; // The simulation will run for 10 seconds.
    float currentTime = 0; // This accumulates the time that has passed.
    float dt = 1; // Each step will take one second.

    InitializeRigidBodies();
    PrintRigidBodies();

    while (currentTime < totalSimulationTime) {
        sleep(dt);

        for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
            RigidBody *rigidBody = &rigidBodies[i];
            ComputeForceAndTorque(rigidBody);
            Vector2 linearAcceleration = (Vector2){rigidBody->force.x / rigidBody->shape.mass, rigidBody->force.y / rigidBody->shape.mass};
            rigidBody->linearVelocity.x += linearAcceleration.x * dt;
            rigidBody->linearVelocity.y += linearAcceleration.y * dt;
            rigidBody->position.x += rigidBody->linearVelocity.x * dt;
            rigidBody->position.y += rigidBody->linearVelocity.y * dt;
            float angularAcceleration = rigidBody->torque / rigidBody->shape.momentOfInertia;
            rigidBody->angularVelocity += angularAcceleration * dt;
            rigidBody->angle += rigidBody->angularVelocity * dt;
        }

        PrintRigidBodies();
        currentTime += dt;
    }
}



int main()
{
    RunRigidBodySimulation();
    return 0;
}
