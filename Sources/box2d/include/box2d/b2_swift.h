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

// b2Shape

b2PolygonShape* b2PolygonShape_create() {
    return new b2PolygonShape();
}

void b2Polygon_delete(b2PolygonShape *shape) {
    delete shape;
}

b2CircleShape* b2CircleShape_create() {
    return new b2CircleShape();
}

void b2CircleShape_delete(b2CircleShape *shape) {
    delete shape;
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

void b2World_delete(b2World *world) {
    delete world;
}

// b2World end
