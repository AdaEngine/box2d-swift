// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "b2_body.h"
#include "b2_shape.h"
#include "b2_joint.h"
#include "b2_world.h"

// b2Body

const b2Vec2& b2Body_GetPosition(const b2Body *body) {
    return body->GetPosition();
}

float b2Body_GetAngle(const b2Body *body) {
    return body->GetAngle();
}

b2Fixture* b2Body_GetFixtureList(b2Body *body) {
    return body->GetFixtureList();
}

b2Fixture* b2Body_CreateFixture(b2Body *body, const b2FixtureDef* def) {
    return body->CreateFixture(def);
}

void b2Body_DestroyFixture(b2Body *body, b2Fixture* fixture) {
    body->DestroyFixture(fixture);
}

void b2Body_SetTransform(b2Body *body, const b2Vec2& position, float angle) {
    body->SetTransform(position, angle);
}

void b2Body_SetLinearVelocity(b2Body *body, const b2Vec2& v) {
    body->SetLinearVelocity(v);
}

const b2Vec2& b2Body_GetWorldCenter(const b2Body *body) {
    return body->GetWorldCenter();
}

const b2Vec2& b2Body_GetLocalCenter(const b2Body *body) {
    return body->GetLocalCenter();
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void b2Body_ApplyForce(b2Body *body, const b2Vec2& force, const b2Vec2& point, bool wake) {
    body->ApplyForce(force, point, wake);
}

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
void b2Body_ApplyForceToCenter(b2Body *body, const b2Vec2& force, bool wake) {
    body->ApplyForceToCenter(force, wake);
}

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
void b2Body_ApplyTorque(b2Body *body, float torque, bool wake) {
    body->ApplyTorque(torque, wake);
}

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void b2Body_ApplyLinearImpulse(b2Body *body, const b2Vec2& impulse, const b2Vec2& point, bool wake) {
    body->ApplyLinearImpulse(impulse, point, wake);
}

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param wake also wake up the body
void b2Body_ApplyLinearImpulseToCenter(b2Body *body, const b2Vec2& impulse, bool wake) {
    body->ApplyLinearImpulseToCenter(impulse, wake);
}

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
void b2Body_ApplyAngularImpulse(b2Body *body, float impulse, bool wake) {
    body->ApplyAngularImpulse(impulse, wake);
}

/// Get the world linear velocity of a world point attached to this body.
/// @param worldPoint a point in world coordinates.
/// @return the world velocity of a point.
b2Vec2 b2Body_GetLinearVelocityFromWorldPoint(const b2Body *body, const b2Vec2& worldPoint) {
    return body->GetLinearVelocityFromWorldPoint(worldPoint);
}

/// Get the world velocity of a local point.
/// @param localPoint a point in local coordinates.
/// @return the world velocity of a point.
b2Vec2 b2Body_GetLinearVelocityFromLocalPoint(const b2Body *body, const b2Vec2& localPoint) {
    return body->GetLinearVelocityFromLocalPoint(localPoint);
}

/// Get the linear velocity of the center of mass.
/// @return the linear velocity of the center of mass.
const b2Vec2& b2Body_GetLinearVelocity(const b2Body *body) {
    return body->GetLinearVelocity();
}

b2BodyUserData& b2Body_GetUserData(b2Body *body) {
    return body->GetUserData();
}

b2BodyDef* b2BodyDef_create() {
    return new b2BodyDef();
}

// end b2Body

// b2Shape

b2PolygonShape* b2PolygonShape_create() {
    return new b2PolygonShape();
}

b2CircleShape* b2CircleShape_create() {
    return new b2CircleShape();
}

const b2Shape* b2Shape_unsafeCast(void *shape) {
    return (const b2Shape *)shape;
}

float& b2Shape_GetRadius(b2Shape *shape) {
    return shape->m_radius;
}

// b2Shape end

// b2Joint

const b2JointDef* b2JointDef_unsafeCast(void *joint) {
    return (const b2JointDef *)joint;
}

// b2Joint end

// b2ContactListner

class ContactListener2D: public b2ContactListener {
public:
    ContactListener2D(const void *userData): m_UserData(userData) {}
    
    ~ContactListener2D() {
        
    }
    
    virtual void BeginContact(b2Contact* contact) override {
        m_BeginContact(m_UserData, contact);
    }
    
    virtual void EndContact(b2Contact* contact) override {
        m_EndContact(m_UserData, contact);
    }
    
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. disable contact).
    /// A copy of the old manifold is provided so that you can detect changes.
    /// Note: this is called only for awake bodies.
    /// Note: this is called even when the number of contact points is zero.
    /// Note: this is not called for sensors.
    /// Note: if you set the number of contact points to zero, you will not
    /// get an EndContact callback. However, you may get a BeginContact callback
    /// the next step.
    virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override {
        m_PreSolve(m_UserData, contact, oldManifold);
    }
    
    /// This lets you inspect a contact after the solver is finished. This is useful
    /// for inspecting impulses.
    /// Note: the contact manifold does not include time of impact impulses, which can be
    /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    /// in a separate data structure.
    /// Note: this is only called for contacts that are touching, solid, and awake.
    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override {
        m_PostSolve(m_UserData, contact, impulse);
    }
    
    void(*m_BeginContact)(const void *userData, b2Contact* contact);
    void(*m_EndContact)(const void *userData, b2Contact* contact);
    void(*m_PreSolve)(const void *userData, b2Contact* contact, const b2Manifold* oldManifold);
    void(*m_PostSolve)(const void *userData, b2Contact* contact, const b2ContactImpulse* impulse);
    
private:
    const void* m_UserData;
};

ContactListener2D* ContactListener2D_create(const void *userData) {
    return new ContactListener2D(userData);
}

b2ContactListener* b2ContactListener_unsafeCast(void *ptr) {
    return (b2ContactListener*)ptr;
}

// b2ContactListner end

// b2World

b2World* b2World_create(const b2Vec2& gravity) {
    return new b2World(gravity);
}

b2Vec2 b2World_GetGravity(const b2World* world) {
    return world->GetGravity();
}

void b2World_SetGravity(b2World* world, const b2Vec2& gravity) {
    world->SetGravity(gravity);
}

/// Register a contact event listener. The listener is owned by you and must
/// remain in scope.
void b2World_SetContactListener(b2World* world, b2ContactListener* listener) {
    world->SetContactListener(listener);
}

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
void b2World_Step(
                  b2World* world,
                  float timeStep,
                  int32 velocityIterations,
                  int32 positionIterations) {
    world->Step(timeStep, velocityIterations, positionIterations);
}

/// Create a rigid body given a definition. No reference to the definition
/// is retained.
/// @warning This function is locked during callbacks.
b2Body* b2World_CreateBody(b2World* world, const b2BodyDef* def) {
    return world->CreateBody(def);
}

/// Destroy a rigid body given a definition. No reference to the definition
/// is retained. This function is locked during callbacks.
/// @warning This automatically deletes all associated shapes and joints.
/// @warning This function is locked during callbacks.
void b2World_DestroyBody(b2World* world, b2Body* body) {
    world->DestroyBody(body);
}

/// Create a joint to constrain bodies together. No reference to the definition
/// is retained. This may cause the connected bodies to cease colliding.
/// @warning This function is locked during callbacks.
b2Joint* b2World_CreateJoint(b2World* world, const b2JointDef* def) {
    return world->CreateJoint(def);
}

/// Destroy a joint. This may cause the connected bodies to begin colliding.
/// @warning This function is locked during callbacks.
void b2World_DestroyJoint(b2World* world, b2Joint* joint) {
    world->DestroyJoint(joint);
}

// b2World end
