// ---------------------------------------------------------------------------
// flow_world.cpp : MAGNET FLOW の物理とルール（flow_world.h）
// ---------------------------------------------------------------------------
#include "flow/flow_world.h"

#include <algorithm>
#include <cmath>

namespace flow {

float momentForGamma(float gamma, float mass, float contactDist) {
    return momentFromGamma(gamma, mass, contactDist, G_ACC);
}

// +y を dir に向ける回転
static Quat rotateYTo(const Vec3& dir) {
    const Vec3  d = normalize(dir);
    const float c = dot(Vec3{0, 1, 0}, d);
    if (c > 0.9999f)  return Quat{};
    if (c < -0.9999f) return Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
    return Quat::fromAxisAngle(cross(Vec3{0, 1, 0}, d), std::acos(clampf(c, -1.0f, 1.0f)));
}

static bool insideBox(const Vec3& p, const Vec3& lo, const Vec3& hi) {
    return p.x >= lo.x && p.y >= lo.y && p.z >= lo.z && p.x <= hi.x && p.y <= hi.y && p.z <= hi.z;
}

// 物の表面でいちばん近い点 → p の向き r と距離 d（中に入っていれば中心からの向き、d = 0）
static void surfaceTo(const RigidBody* b, const Vec3& p, Vec3& r, float& d) {
    if (b->shape.type == ShapeType::Sphere) {
        const Vec3  dv = p - b->position;
        const float l  = length(dv);
        r = l > 1e-6f ? dv / l : Vec3{0, 1, 0};
        d = l - b->shape.radius;
    } else {
        const Mat3 R = b->orientation.toMat3();
        Vec3 q = R.transposed() * (p - b->position);
        for (int i = 0; i < 3; ++i) q[i] = clampf(q[i], -b->shape.halfExtents[i], b->shape.halfExtents[i]);
        const Vec3  diff = p - (b->position + R * q);
        const float l    = length(diff);
        if (l > 1e-6f) { r = diff / l; d = l; }
        else           { r = normalize(p - b->position); d = 0.0f; }
    }
}

// 扉の開き具合 t（0..1、なめらかにした後）の位置と姿勢
static void placeGate(Gate& g, float t) {
    if (g.hinged) {   // ヒンジのまわりに回す（扉に付けた物も一緒に）
        const Quat R = Quat::fromAxisAngle(g.hingeAxis, g.openAngle * t);
        g.door->position    = g.pivot + R.rotate(g.closedPos - g.pivot);
        g.door->orientation = R * g.closedRot;
        for (Attached& a : g.attached) {
            a.body->position    = g.pivot + R.rotate(a.closedPos - g.pivot);
            a.body->orientation = R * a.closedRot;
        }
    } else {
        const Vec3 off = (g.openPos - g.closedPos) * t;
        g.door->position = g.closedPos + off;
        for (Attached& a : g.attached) a.body->position = a.closedPos + off;
    }
}

// ---------------------------------------------------------------------------
// 組み立て
// ---------------------------------------------------------------------------
void FlowWorld::reset() {
    world.clear();
    world.gravity = Vec3{0, -G_ACC * params.gravity, 0};
    world.substeps = 1;
    player   = nullptr;
    polarity = +1;
    state    = State::Ready;
    stats    = FlowStats{};
    events   = FlowEvents{};
    kinds.clear();
    charge.clear();
    poleAxis.clear();
    flash.clear();
    moved.clear();
    used.clear();
    coreOf.clear();
    swingOf.clear();
    steelOf.clear();
    padOf.clear();
    uniformPole.clear();
    gates.clear();
    cores.clear();
    animators.clear();
    swings.clear();
    steels.clear();
    rotors.clear();
    pads.clear();
    zones.clear();
    needles.clear();
    cyclotrons.clear();
    rings.clear();
    chains.clear();
    buildZone = -1;
    links.clear();
    trail.clear();
    trailBreaks.clear();
    marks.clear();
    inputs.clear();
    netForce       = Vec3{0, 0, 0};
    vortexForce    = Vec3{0, 0, 0};
    startPos       = Vec3{0, 1, 0};
    startPolarity  = +1;
    launchVelocity = Vec3{0, 0, 0};
    viewDir        = Vec3{0, -0.3f, -1};
    overview       = false;
    viewCenter     = Vec3{0, 0, 0};
    viewDist       = 30.0f;
    killY          = -10.0f;
    hasGoal        = false;
    magnetTotal    = 0;
    failTimer      = 0.0f;
    stepCount      = 0;
    prevSpeed      = 0.0f;
    lastCapture    = -1;
    orbitCore      = -1;
    orbitAngle     = 0.0f;
    orbitPrev      = 0.0f;
    swingActive    = -1;
    swingAngle     = 0.0f;
    swingPrev      = 0.0f;
    comboClock     = 0.0f;
    checkpoint      = -1;
    checkpointTotal = 0;
    zone            = -1;
    seatedPad       = -1;
    lastSeat        = -1;
    practice        = false;
    splits.clear();
    resets.clear();
    zoneStart.clear();
}

void FlowWorld::tag(RigidBody* b, Kind k, float q, const Vec3& axisLocal) {
    if ((int)kinds.size() <= b->id) {
        kinds.resize(b->id + 1, Kind::Other);
        charge.resize(b->id + 1, 0.0f);
        poleAxis.resize(b->id + 1, Vec3{0, 1, 0});
        flash.resize(b->id + 1, 0.0f);
        moved.resize(b->id + 1, 0);
        used.resize(b->id + 1, 0);
        coreOf.resize(b->id + 1, -1);
        swingOf.resize(b->id + 1, -1);
        steelOf.resize(b->id + 1, -1);
        padOf.resize(b->id + 1, -1);
        uniformPole.resize(b->id + 1, 0);
    }
    kinds[b->id]    = k;
    charge[b->id]   = q;
    poleAxis[b->id] = axisLocal;
}

RigidBody* FlowWorld::addSolid(const Vec3& pos, const Vec3& he, const Quat& q, float friction, float restitution) {
    RigidBody* b = world.createBox(pos, he, 0.0f);
    b->orientation = q;
    b->friction    = friction;
    b->restitution = restitution;
    tag(b, Kind::Solid, 0.0f);
    return b;
}

RigidBody* FlowWorld::addFixedMagnetSphere(const Vec3& pos, float radius, const Vec3& northDir, float q,
                                           float worldMoment) {
    // createMagnet はモーメントをローカル +y に置く（向きは姿勢で表す）。0 のときも向きを決めておく
    RigidBody* b = world.createMagnet(pos, radius, 0.0f, normalize(northDir) * worldMoment);
    b->orientation = rotateYTo(northDir);
    b->friction = 1.0f; b->restitution = 0.0f;
    tag(b, Kind::FixedMagnet, q);
    return b;
}

RigidBody* FlowWorld::addFixedMagnetBox(const Vec3& pos, const Vec3& he, const Quat& q, float c, float worldMoment) {
    RigidBody* b = world.createMagnetBox(pos, he, 0.0f, Vec3{0, worldMoment, 0}, q);
    b->friction = 1.0f; b->restitution = 0.0f;
    tag(b, Kind::FixedMagnet, c);
    return b;
}

RigidBody* FlowWorld::addMovableMagnetBox(const Vec3& pos, float half, const Vec3& northDir, float mass, float q,
                                          float worldMoment) {
    RigidBody* b = world.createMagnetBox(pos, {half, half, half}, mass, Vec3{0, worldMoment, 0}, rotateYTo(northDir));
    b->friction = 0.3f; b->restitution = 0.1f;
    tag(b, Kind::MovableMagnet, q);
    return b;
}

RigidBody* FlowWorld::addMovableMagnetSphere(const Vec3& pos, float radius, const Vec3& northDir, float mass, float q,
                                             float worldMoment) {
    RigidBody* b = world.createMagnet(pos, radius, mass, normalize(northDir) * worldMoment);
    b->orientation = rotateYTo(northDir);
    b->friction = 0.3f; b->restitution = 0.2f; b->rollingFriction = 0.02f;
    tag(b, Kind::MovableMagnet, q);
    return b;
}

RigidBody* FlowWorld::addIronSphere(const Vec3& pos, float radius, float mass, float q) {
    RigidBody* b = world.createSoftMagnet(pos, radius, mass, sphereSusceptibility(radius, 1000.0f), 0.0f);
    b->friction = 0.3f; b->restitution = 0.2f; b->rollingFriction = 0.02f;
    tag(b, Kind::Iron, q);
    return b;
}

RigidBody* FlowWorld::addIronBox(const Vec3& pos, const Vec3& he, float mass, const Quat& q, float c) {
    RigidBody* b = world.createSoftMagnetBox(pos, he, mass, boxSusceptibility(he, 1000.0f), 0.0f, q);
    b->friction = 0.4f; b->restitution = 0.1f;
    tag(b, Kind::Iron, c);
    return b;
}

Gate& FlowWorld::addGate(const Vec3& pos, const Vec3& he, const Vec3& openOffset, const Vec3& triggerLo,
                         const Vec3& triggerHi) {
    Gate g;
    g.door = addSolid(pos, he);
    kinds[g.door->id] = Kind::Door;
    g.closedPos = pos;
    g.openPos   = pos + openOffset;
    g.lo = triggerLo;
    g.hi = triggerHi;
    g.zone = buildZone;
    gates.push_back(g);
    return gates.back();
}

Gate& FlowWorld::addHingedGate(const Vec3& pos, const Vec3& he, const Quat& q, const Vec3& pivot, const Vec3& axis,
                               float angle, const Vec3& triggerLo, const Vec3& triggerHi) {
    Gate& g = addGate(pos, he, Vec3{0, 0, 0}, triggerLo, triggerHi);
    g.door->orientation = q;
    g.closedRot = q;
    g.hinged    = true;
    g.pivot     = pivot;
    g.hingeAxis = normalize(axis);
    g.openAngle = angle;
    return g;
}

void FlowWorld::attachToGate(Gate& g, RigidBody* b) { g.attached.push_back({b, b->position, b->orientation}); }

Core& FlowWorld::addCore(const Vec3& pos, float radius, int pole, const Vec3& axis, float q) {
    // 物どうしの双極子は持たない（プレイヤーにだけ効く）。表面はすべりやすく、周回をじゃましない
    RigidBody* b = world.createSphere(pos, radius, 0.0f);
    b->friction = 0.0f; b->restitution = 0.0f; b->rollingFriction = 0.0f;
    tag(b, Kind::Core, q);
    Core c;
    c.body = b;
    c.pole = pole;
    c.axis = normalize(axis);
    coreOf[b->id] = (int)cores.size();
    cores.push_back(c);
    return cores.back();
}

Core& FlowWorld::addPole(const Vec3& pos, float radius, int pole, float q) {
    Core& c = addCore(pos, radius, pole, Vec3{0, 1, 0}, q);
    c.plain = true;
    c.reach = 0.0f;
    c.push = c.lock = c.hold = 0.0f;
    c.repel = 1.0f;
    c.body->friction = 0.6f;
    return c;
}

Animator& FlowWorld::animate(RigidBody* b) {
    Animator a;
    a.body        = b;
    a.basePos     = b->position;
    a.baseRot     = b->orientation;
    a.baseCharge  = b->id < (int)charge.size() ? charge[b->id] : 1.0f;
    a.magnetIndex = world.magnets().indexOf(b);
    if (a.magnetIndex >= 0) a.basePermanent = world.magnets().bodies()[a.magnetIndex].permanentLocal;
    a.zone = buildZone;
    animators.push_back(a);
    return animators.back();
}

void FlowWorld::attachToAnimator(Animator& a, RigidBody* b) { a.attached.push_back({b, b->position, b->orientation}); }

Swing& FlowWorld::addSwing(const Vec3& pos, float radius, int pole, float length, const Vec3& planeN, float q) {
    RigidBody* b = world.createSphere(pos, radius, 0.0f);
    b->friction = 0.2f; b->restitution = 0.2f;
    tag(b, Kind::Anchor, q);
    Swing s;
    s.body   = b;
    s.pole   = pole;
    s.length = length;
    s.planeN = lengthSq(planeN) > 1e-8f ? normalize(planeN) : Vec3{0, 0, 0};
    swingOf[b->id] = (int)swings.size();
    swings.push_back(s);
    return swings.back();
}

Steel& FlowWorld::addSteelSphere(const Vec3& pos, float radius, float mass, float moment, float q) {
    RigidBody* b = world.createSoftMagnet(pos, radius, mass, sphereSusceptibility(radius, 1000.0f), 0.0f);
    b->friction = 0.3f; b->restitution = 0.2f; b->rollingFriction = 0.02f;
    tag(b, Kind::Iron, q);
    Steel s;
    s.body        = b;
    s.moment      = moment;
    s.magnetIndex = world.magnets().indexOf(b);
    steelOf[b->id] = (int)steels.size();
    steels.push_back(s);
    return steels.back();
}

Steel& FlowWorld::addSteelBox(const Vec3& pos, const Vec3& he, float mass, float moment, const Quat& q, float c) {
    RigidBody* b = world.createSoftMagnetBox(pos, he, mass, boxSusceptibility(he, 1000.0f), 0.0f, q);
    b->friction = 0.6f; b->restitution = 0.1f;
    tag(b, Kind::Iron, c);
    Steel s;
    s.body        = b;
    s.moment      = moment;
    s.magnetIndex = world.magnets().indexOf(b);
    steelOf[b->id] = (int)steels.size();
    steels.push_back(s);
    return steels.back();
}

Rotor& FlowWorld::addRotor(RigidBody* b, const Vec3& pivot, const Vec3& axis, float lo, float hi, float inertia) {
    Rotor r;
    r.body    = b;
    r.pivot   = pivot;
    r.axis    = normalize(axis);
    r.basePos = b->position;
    r.baseRot = b->orientation;
    r.lo      = lo;
    r.hi      = hi;
    r.inertia = inertia;
    r.zone    = buildZone;
    if (b->id < (int)kinds.size() && kinds[b->id] == Kind::Solid) kinds[b->id] = Kind::Door;
    rotors.push_back(r);
    return rotors.back();
}

void FlowWorld::attachToRotor(Rotor& r, RigidBody* b) { r.attached.push_back({b, b->position, b->orientation}); }

Pad& FlowWorld::addPad(const Vec3& pos, const Vec3& he, const Vec3& launch, int launchPolarity, int cp) {
    RigidBody* b = world.createBox(pos, he, 0.0f);
    b->friction = 1.0f; b->restitution = 0.0f;
    tag(b, Kind::Pad, 1.0f);
    Pad p;
    p.body       = b;
    p.seat       = pos + Vec3{0, he.y + 0.5f, 0};
    p.seatRadius = std::max(1.2f, std::min(he.x, he.z) + 0.5f);
    p.launch     = launch;
    p.polarity   = launchPolarity;
    p.checkpoint = cp;
    p.zone       = buildZone;
    padOf[b->id] = (int)pads.size();
    pads.push_back(p);
    checkpointTotal = std::max(checkpointTotal, cp + 1);
    return pads.back();
}

Pad& FlowWorld::addCheckpoint(const Vec3& seat, float radius, int pol, int cp) {
    Pad p;
    p.seat       = seat;
    p.seatRadius = radius;
    p.polarity   = pol;
    p.checkpoint = cp;
    p.reach      = 0.0f;
    p.zone       = buildZone;
    pads.push_back(p);
    checkpointTotal = std::max(checkpointTotal, cp + 1);
    return pads.back();
}

Zone& FlowWorld::addZone(const std::string& name, const std::string& hint, const Vec3& lo, const Vec3& hi, float ky) {
    Zone z;
    z.name  = name;
    z.hint  = hint;
    z.lo    = lo;
    z.hi    = hi;
    z.killY = ky;
    zones.push_back(z);
    buildZone = (int)zones.size() - 1;
    return zones.back();
}

Cyclotron& FlowWorld::addCyclotron(const Vec3& center, const Vec3& axis, const Vec3& gapN) {
    Cyclotron c;
    c.center  = center;
    c.axis    = normalize(axis);
    c.gapN    = normalize(gapN - c.axis * dot(gapN, c.axis));
    c.portDir = c.gapN;
    cyclotrons.push_back(c);
    return cyclotrons.back();
}

GaussRing& FlowWorld::addGaussRing(const Vec3& center, const Vec3& axis, float radius, int pole) {
    GaussRing g;
    g.center = center;
    g.axis   = normalize(axis);
    g.radius = radius;
    g.pole   = pole;
    rings.push_back(g);
    return rings.back();
}

BallChain& FlowWorld::addBallChain(const Vec3& topPos, float topRadius, int n, float r, float mass, int northDown,
                                   float q, float gamma) {
    // 球どうし: 接触した 2 個の引力が gamma x 重さ。天井の磁石は、いちばん上の球をその 3 倍で引く
    //   （同じ軸に並んだ 2 つの双極子の引力は 6 m1 m2 / d^4）
    const Vec3  north = Vec3{0, -1.0f, 0} * (float)northDown;
    const float mb = momentForGamma(gamma, mass, 2.0f * r);
    const float d  = topRadius + r;
    const float mt = 3.0f * mb * std::pow(d / (2.0f * r), 4.0f);
    BallChain ch;
    ch.top  = addFixedMagnetSphere(topPos, topRadius, north, 0.0f, mt);   // プレイヤーには効かない
    uniformPole[ch.top->id] = (signed char)northDown;
    for (int i = 0; i < n; ++i) {
        const Vec3 pos = topPos - Vec3{0, d + 2.0f * r * (float)i, 0};
        // 並んだ双極子の鎖は、外から見ると両端に極がある棒磁石と同じ。プレイヤーには先の球だけが、
        // 先の極（下の面。曲がっても変わらない）として働く
        RigidBody* b = addMovableMagnetSphere(pos, r, north, mass, i == n - 1 ? q : 0.0f, mb);
        uniformPole[b->id] = (signed char)northDown;
        ch.balls.push_back(b);
    }
    chains.push_back(ch);
    return chains.back();
}

void FlowWorld::finishBuild() {
    player = world.createSphere(startPos, 0.5f, 1.0f);
    // 磁石に強く押し付けられたときに止まって「吸い付く」ように、転がりにくく跳ねにくくする
    player->friction        = 0.8f;
    player->restitution     = 0.1f;
    player->rollingFriction = 0.3f;
    tag(player, Kind::Player, 0.0f);
    polarity = startPolarity;
    magnetTotal = 0;   // 鎖の球と天井の磁石（プレイヤーに効かない）は数えない
    for (size_t i = 0; i < kinds.size(); ++i) magnetTotal += isMagnet(kinds[i]) && !(uniformPole[i] != 0 && charge[i] == 0.0f);
    zoneStart.assign(zones.size(), -1.0f);
    for (const Pad& p : pads) {   // 最初の発射台から始める
        if (p.checkpoint != 0) continue;
        checkpoint = 0;
        splits     = {0.0f};
        zone       = p.zone;
        if (zone >= 0) zoneStart[zone] = 0.0f;
    }
    updateZone();
    updateAnimators(0.0f);
    if (!rotors.empty()) updateRotors(0.0f);   // 組み立てで決めた角度に置く
    world.magnets().updateMoments(20);
    float s = 1.0f;
    netForce = computePlayerForce(&links, &s) * s;
}

void FlowWorld::restart() {
    const bool played = trail.size() > 1;
    std::vector<Vec3>       keepTrail  = played ? trail : prevTrail;
    std::vector<int>        keepBreaks = played ? trailBreaks : prevBreaks;
    std::vector<SwitchMark> keepMarks  = played ? marks : prevMarks;
    std::vector<float>      keepInputs = played ? inputs : prevInputs;
    if (played) ++retries;
    reset();
    if (builder) builder(*this);
    prevTrail  = std::move(keepTrail);
    prevBreaks = std::move(keepBreaks);
    prevMarks  = std::move(keepMarks);
    prevInputs = std::move(keepInputs);
}

// ---------------------------------------------------------------------------
// チェックポイント（大きなマップ）
// ---------------------------------------------------------------------------
void FlowWorld::placeAtCheckpoint(int cp) {
    for (const Pad& p : pads) {
        if (p.checkpoint != cp) continue;
        player->position        = p.seat;
        player->velocity        = Vec3{0, 0, 0};
        player->angularVelocity = Vec3{0, 0, 0};
        polarity = p.hold != 0 ? p.hold : p.polarity;
        zone     = p.zone;
        if (zone >= 0 && zone < (int)zoneStart.size()) zoneStart[zone] = stats.time;   // 区間の時計をやり直す
        openBehind(zone);
        break;
    }
    updateZone();
    updateAnimators(0.0f);
    float s = 1.0f;
    netForce = computePlayerForce(&links, &s) * s;
}

// 戻ったチェックポイントより前の区間の扉と板は、開いた（倒れた）状態にしておく
void FlowWorld::openBehind(int zoneIndex) {
    if (zoneIndex < 0) return;
    for (Gate& g : gates) {
        if (g.zone < 0 || g.zone >= zoneIndex) continue;
        g.opening = true;
        g.open    = 1.0f;
        placeGate(g, 1.0f);
    }
    for (Rotor& r : rotors) {
        if (r.zone < 0 || r.zone >= zoneIndex) continue;
        r.angle = r.hi;
        r.omega = 0.0f;
        r.reachedHi = true;
    }
    updateRotors(0.0f);
}

void FlowWorld::respawn() {
    // 走りの記録（時間・入力・軌跡など）はそのまま残し、マップだけを組み直す
    const FlowStats         keepStats  = stats;
    std::vector<float>      keepInputs = std::move(inputs), keepSplits = std::move(splits), keepResets = std::move(resets);
    std::vector<float>      keepZone   = zoneStart;
    std::vector<Vec3>       keepTrail  = std::move(trail);
    std::vector<int>        keepBreaks = std::move(trailBreaks);
    std::vector<SwitchMark> keepMarks  = std::move(marks);
    std::vector<char>       keepUsed   = used;
    const int  cp = checkpoint, keepStep = stepCount;
    const bool keepPractice = practice;
    reset();
    if (builder) builder(*this);
    stats       = keepStats;
    inputs      = std::move(keepInputs);
    splits      = std::move(keepSplits);
    resets      = std::move(keepResets);
    trail       = std::move(keepTrail);
    trailBreaks = std::move(keepBreaks);
    marks       = std::move(keepMarks);
    if (keepZone.size() == zoneStart.size()) zoneStart = keepZone;
    for (size_t i = 0; i < used.size() && i < keepUsed.size(); ++i) used[i] = keepUsed[i];
    stepCount  = keepStep;
    practice   = keepPractice;
    checkpoint = cp;
    stats.combo = 0;
    comboClock  = COMBO_GAP + 1.0f;
    placeAtCheckpoint(cp);
    state = State::Running;
    trailBreaks.push_back((int)trail.size());
    events.respawned = true;
}

void FlowWorld::resetToCheckpoint() {
    if (!bigMap() || state != State::Running || checkpoint < 0) return;
    resets.push_back(stats.time);
    respawn();
}

void FlowWorld::warpTo(int cp) {
    reset();
    if (builder) builder(*this);
    prevTrail.clear();
    prevBreaks.clear();
    prevMarks.clear();
    prevInputs.clear();
    if (cp <= 0 || !bigMap()) return;
    practice   = true;
    checkpoint = cp;
    splits.assign(cp + 1, 0.0f);
    placeAtCheckpoint(cp);
}

// ---------------------------------------------------------------------------
// 遊ぶ
// ---------------------------------------------------------------------------
void FlowWorld::press() {
    if (!player) return;
    // 座っている発射台（引く力を止めていないもの）
    auto seated = [&]() {
        for (int i = 0; i < (int)pads.size(); ++i)
            if (pads[i].cooldown <= 0.0f && padActive(i) && length(player->position - pads[i].seat) < pads[i].seatRadius)
                return i;
        return -1;
    };
    auto fire = [&](int i) {   // 発射台が決めた初速と極性で飛ばす（座る位置にそろえてから。区間の始めを毎回同じにする）
        Pad& p = pads[i];
        player->position        = p.seat;
        player->angularVelocity = Vec3{0, 0, 0};
        player->velocity = p.launch;
        if (polarity != p.polarity) {
            polarity = p.polarity;
            events.switched = true;
            marks.push_back({player->position, polarity});
        }
        p.cooldown = 1.0f;
        events.launched = true;
        lastCapture = -1;
    };
    if (state == State::Ready) {
        state = State::Running;
        events.started = true;
        trail.push_back(player->position);
        inputs.push_back(0.0f);
        const int sp = seated();
        if (sp >= 0) fire(sp);
        else         player->velocity = launchVelocity;
        return;
    }
    if (state != State::Running) return;
    const int sp = seated();
    if (sp >= 0) {
        fire(sp);
        inputs.push_back(stats.time);
        return;
    }
    float s = 1.0f;
    const Vec3 before = computePlayerForce(nullptr, &s) * s;
    polarity = -polarity;
    const Vec3 after = computePlayerForce(nullptr, &s) * s;
    ++stats.switches;
    marks.push_back({player->position, polarity});
    inputs.push_back(stats.time);
    events.switched = true;
    // 反発に切り替わった（合力の向きが逆になり、十分大きい）: 射出
    const float cap = params.forceCap * G_ACC / player->invMass;
    if (dot(before, after) < 0.0f && length(after) > 0.3f * cap) {
        events.kick = true;
        magneticEvent();
        lastCapture = -1;   // 同じ磁石にもう一度吸われても「吸着」に数える
    }
}

void FlowWorld::begin() {
    if (!player || state != State::Ready) return;
    state = State::Running;
    events.started = true;
    trail.push_back(player->position);
    inputs.push_back(0.0f);
}

void FlowWorld::magneticEvent() {
    ++stats.events;
    stats.combo = (comboClock <= COMBO_GAP && stats.combo > 0) ? stats.combo + 1 : 1;
    stats.bestCombo = std::max(stats.bestCombo, stats.combo);
    comboClock = 0.0f;
}

Vec3 FlowWorld::forceAt(const Vec3& p, int pol, std::vector<PlayerLink>* out) const {
    if (out) out->clear();
    Vec3 total{0, 0, 0};
    const float rp = player ? player->shape.radius : 0.5f;
    const float a2 = params.soft * params.soft, Rc = params.range;
    for (const auto& bp : world.getBodies()) {
        const RigidBody* b = bp.get();
        const Kind       k = kindOf(b);
        if (k != Kind::FixedMagnet && k != Kind::MovableMagnet && k != Kind::Iron && k != Kind::Core &&
            k != Kind::Anchor && k != Kind::Pad)
            continue;
        // 物の表面でいちばん近い点 → プレイヤー
        Vec3  r;
        float d;
        surfaceTo(b, p, r, d);
        d = std::max(0.0f, d - rp);   // プレイヤーの表面から
        if (d >= Rc) continue;
        const float w = 1.0f - (d / Rc) * (d / Rc);
        const float g = w * w / (d * d + a2);
        Vec3 F;
        if (k == Kind::Iron) {
            F = r * (-params.Kiron * charge[b->id] * g);
            const int si = steelOf[b->id];
            if (si >= 0) {   // 磁化した鋼: 磁化の向き M の磁石として働く（大きさ |M|）
                const Steel& st = steels[si];
                F += r * ((float)pol * dot(st.worldM(), r) * st.strength * charge[b->id] * params.K * g);
            }
        } else if (k == Kind::Core) {   // 表面のどこでも同じ極が外を向いている。反発は弱め
            const Core& c = cores[coreOf[b->id]];
            const int   s = pol * c.pole;
            F = r * ((float)s * (s > 0 ? c.repel : 1.0f) * charge[b->id] * params.K * g);
        } else if (k == Kind::Anchor) {   // 錨: 同極なら反発、異極なら綱がつながるまで引く（つながったら綱が受け持つ）
            const Swing& sw = swings[swingOf[b->id]];
            const int    s  = pol * sw.pole;
            const float  f  = s > 0 ? sw.repel : (sw.engaged ? 0.0f : sw.pull);
            F = r * ((float)s * f * charge[b->id] * params.K * g);
        } else if (k == Kind::Pad) {      // 発射台: どちらの極でも、座る位置（台の中央の上）へ引く
            const Pad& pd = pads[padOf[b->id]];
            const Vec3  rs = p - pd.seat;
            const float L  = length(rs);
            // 上面より上にいるときだけ（横や下に吸い付かない）。座る位置では 0（ばねのように中央へ寄せる）
            if (pd.cooldown > 0.0f || L >= pd.reach || L < 1e-4f || p.y < pd.seat.y - 0.7f) continue;
            r = rs / L;
            const float wp = 1.0f - (L / pd.reach) * (L / pd.reach);
            F = r * (-params.Kiron * charge[b->id] * wp * wp * L / (L * L + a2));
        } else {
            const float c = uniformPole[b->id] != 0 ? (float)uniformPole[b->id] : dot(b->orientation.rotate(poleAxis[b->id]), r);
            F = r * ((float)pol * c * charge[b->id] * params.K * g);
        }
        total += F;
        if (out) out->push_back({b->id, F, dot(F, r) < 0.0f});
    }
    return total;
}

Vec3 FlowWorld::computePlayerForce(std::vector<PlayerLink>* out, float* scale) const {
    if (!player) {
        if (out) out->clear();
        if (scale) *scale = 1.0f;
        return Vec3{0, 0, 0};
    }
    const Vec3  total = forceAt(player->position, polarity, out);
    const float cap = params.forceCap * G_ACC / player->invMass;
    const float len = length(total);
    if (scale) *scale = (len > cap) ? cap / len : 1.0f;
    return total;
}

Vec3 FlowWorld::computeVortex() const {
    Vec3 F{0, 0, 0};
    if (!player) return F;
    const Vec3  p = player->position, v = player->velocity;
    const float m = 1.0f / player->invMass;
    for (const Core& c : cores) {
        const Vec3  r = p - c.body->position;
        const float d = std::max(0.0f, length(r) - c.body->shape.radius - player->shape.radius);
        if (d >= c.reach) continue;
        const Vec3 t = cross(c.axis, r);
        if (lengthSq(t) < 1e-8f) continue;   // 軸の上
        const Vec3  tn = normalize(t);
        const float w  = (1.0f - d / c.reach) * (1.0f - d / c.reach);
        const Vec3  off = v - tn * dot(v, tn);   // 回転の向き以外の速度（半径方向と軸方向）
        const Vec3  up  = c.axis * dot(r, c.axis);   // 回転面からのずれ
        F += (tn * (c.push * params.corePush) - off * (c.lock * params.coreLock) - up * c.hold) * (m * w);
    }
    // 磁気振り子: 下を通るときに進む向きへ押し、振れる面に保つ
    for (const Swing& s : swings) {
        if (!s.engaged) continue;
        const Vec3  r = p - s.body->position;
        const float L = length(r);
        if (L < 1e-4f) continue;
        const Vec3  n  = r / L;
        const Vec3  vt = v - n * dot(v, n);
        const float b  = std::max(0.0f, -n.y);   // 真下で 1
        const float gov = std::max(0.0f, 1.0f - length(vt) / s.cruise);
        if (lengthSq(vt) > 0.01f) F += normalize(vt) * (m * s.pump * params.swingPump * b * b * gov);
        if (lengthSq(s.planeN) > 0.0f)
            F -= s.planeN * (m * (s.lock * dot(v, s.planeN) + s.hold * dot(r, s.planeN)));
    }
    // サイクロトロン: 面に浮かせ、半分の中では回し、隙間では極性の向きに押す
    for (const Cyclotron& c : cyclotrons) {
        Vec3  rIn;
        float h;
        if (!cyclotronInside(c, p, &rIn, &h)) continue;
        F += (world.gravity * -1.0f - c.axis * (c.lock * dot(v, c.axis) + c.hold * h)) * m;
        const float u = dot(rIn, c.gapN), au = std::fabs(u);
        if (au < c.gapHalf) continue;   // 隙間の中はまっすぐ
        const float t    = c.fringe > 0.0f ? clampf((au - c.gapHalf) / c.fringe, 0.0f, 1.0f) : 1.0f;
        const float side = u > 0.0f ? 1.0f : -1.0f;
        const Vec3  vIn  = v - c.axis * dot(v, c.axis);
        F += cross(c.axis, vIn) * (m * c.omega * (float)polarity * side * t * t * (3.0f - 2.0f * t));
    }
    // ガウス加速リング: 面へ引き込む（異極）・面から押し出す（同極）。輪の中では軸へ寄せる
    for (const GaussRing& g : rings) {
        const Vec3  d = p - g.center;
        const float x = dot(d, g.axis);
        if (std::fabs(x) >= g.reach) continue;
        const Vec3 rho = d - g.axis * x;
        if (lengthSq(rho) >= g.radius * g.radius) continue;
        const Vec3 vPerp = v - g.axis * dot(v, g.axis);
        F += g.axis * (m * g.accel * (float)(polarity * g.pole) * std::sin(PHYS_PI * x / g.reach)) -
             (rho * g.guide + vPerp * g.guideDamp) * m;
    }
    return F;
}

bool FlowWorld::cyclotronInside(const Cyclotron& c, const Vec3& p, Vec3* rIn, float* h) const {
    const Vec3  r  = p - c.center;
    const float hh = dot(r, c.axis);
    const Vec3  ri = r - c.axis * hh;
    if (rIn) *rIn = ri;
    if (h)   *h   = hh;
    if (std::fabs(hh) > c.height) return false;
    return lengthSq(ri) < c.radius * c.radius && dot(ri, c.portDir) < c.portRadius;   // 出口の側は弦で終わる
}

bool FlowWorld::padActive(int i) const {
    const Pad& p = pads[i];
    if (lengthSq(p.launch) <= 0.0f) return false;
    return p.lockGate < 0 || (p.lockGate < (int)gates.size() && gates[p.lockGate].opening);
}

void FlowWorld::applyPlayerForce() {
    float s = 1.0f;
    const Vec3 total = computePlayerForce(&links, &s);
    netForce    = total * s;
    vortexForce = computeVortex();
    player->force += netForce + vortexForce;
    const auto& bodies = world.getBodies();
    for (PlayerLink& l : links) {
        l.force = l.force * s;
        RigidBody* b = bodies[l.body].get();
        if (!b->isStatic()) b->force -= l.force;   // 反作用
    }
}

void FlowWorld::updateGates(float dt) {
    for (Gate& g : gates) {
        if (!g.opening) {
            bool in = false;
            if (g.rotorTrigger >= 0) {   // 振り子が止め hi に届いた（壁を叩き壊した）
                in = g.rotorTrigger < (int)rotors.size() && rotors[g.rotorTrigger].reachedHi;
            } else {
                for (const auto& bp : world.getBodies()) {
                    const Kind k = kindOf(bp.get());
                    if ((k == Kind::MovableMagnet || k == Kind::Iron) && !bp->isStatic() && insideBox(bp->position, g.lo, g.hi)) {
                        in = true;
                        break;
                    }
                }
            }
            g.held = in ? g.held + dt : 0.0f;
            if (in && g.held >= (g.rotorTrigger >= 0 ? 0.0f : 0.25f)) {
                g.opening = true;
                ++stats.chain;
                events.gateOpened = true;
                magneticEvent();
            }
        }
        if (g.opening && g.open < 1.0f) {
            g.open = std::min(1.0f, g.open + dt / g.openTime);
            placeGate(g, g.open * g.open * (3.0f - 2.0f * g.open));
        }
    }
}

// 動的磁場: 区間の時計で、往復・自転・点滅させる（tOffset はステップの中のサブステップの時刻）
void FlowWorld::updateAnimators(float tOffset) {
    for (Animator& a : animators) {
        const float t = (state == State::Ready ? 0.0f : zoneTime(a.zone) + tOffset) + a.phase;
        Vec3 off{0, 0, 0}, vel{0, 0, 0};
        if (a.travelPeriod > 0.0f) {
            const float w = 2.0f * PHYS_PI / a.travelPeriod;
            off = a.travel * (0.5f - 0.5f * std::cos(w * t));
            vel = a.travel * (0.5f * w * std::sin(w * t));
        }
        float ang = 0.0f, angVel = 0.0f;
        if (a.spinStep > 0.0f) {   // spinStep ごとに半回転（最後の T 秒で回る）
            const float S = a.spinStep, T = std::min(0.5f, 0.5f * S);
            const float c = t / S, n = std::floor(c), f = (c - n) * S;
            const float u = clampf((f - (S - T)) / T, 0.0f, 1.0f);
            ang    = PHYS_PI * (n + u * u * (3.0f - 2.0f * u));
            angVel = (u > 0.0f && u < 1.0f) ? PHYS_PI * 6.0f * u * (1.0f - u) / T : 0.0f;
        } else if (a.spinRate != 0.0f) {
            ang    = a.spinRate * t;
            angVel = a.spinRate;
        }
        const Vec3 axis = normalize(a.spinAxis);
        const Quat R    = Quat::fromAxisAngle(axis, ang);
        const Vec3 w    = axis * angVel;
        a.body->position    = a.basePos + off;
        a.body->orientation = R * a.baseRot;
        a.body->kinematicVelocity        = vel;
        a.body->kinematicAngularVelocity = w;
        for (Attached& at : a.attached) {
            const Vec3 rel = R.rotate(at.closedPos - a.basePos);
            at.body->position    = a.basePos + off + rel;
            at.body->orientation = R * at.closedRot;
            at.body->kinematicVelocity        = vel + cross(w, rel);
            at.body->kinematicAngularVelocity = w;
        }
        if (a.pulsePeriod > 0.0f) {
            const float P = a.pulsePeriod, on = a.pulseOn * P, ramp = 0.08f;
            float tp = std::fmod(t, P);
            if (tp < 0.0f) tp += P;
            a.level      = tp < on ? clampf(std::min(tp, on - tp) / ramp, 0.0f, 1.0f) : 0.0f;
            a.nextToggle = tp < on ? on - tp : P - tp;
            charge[a.body->id] = a.baseCharge * a.level;
            if (a.magnetIndex >= 0) world.magnets().bodies()[a.magnetIndex].permanentLocal = a.basePermanent * a.level;
        }
    }
}

// 磁気トルクで回る板: プレイヤーの磁力の反作用のトルクと重力のトルクで角速度を変える
void FlowWorld::updateRotors(float h) {
    const auto& bodies = world.getBodies();
    for (Rotor& r : rotors) {
        if (h > 0.0f) {
            float tau = 0.0f;
            for (const PlayerLink& l : links) {   // 反作用 -F は、プレイヤーにいちばん近い表面の点に働く
                const RigidBody* b = bodies[l.body].get();
                bool mine = b == r.body;
                for (const Attached& at : r.attached) mine = mine || at.body == b;
                if (!mine) continue;
                Vec3  dir;
                float d;
                surfaceTo(b, player->position, dir, d);
                tau += dot(cross(player->position - dir * d - r.pivot, l.force * -1.0f), r.axis);
            }
            if (r.mass > 0.0f) {
                const Vec3 c = Quat::fromAxisAngle(r.axis, r.angle).rotate(r.com - r.pivot);
                tau += dot(cross(c, world.gravity * r.mass), r.axis);
            }
            const float w0 = r.omega;
            r.omega += tau / r.inertia * h;
            r.omega *= 1.0f / (1.0f + r.damping * h);
            r.angle += r.omega * h;
            if (r.angle >= r.hi) {
                r.angle = r.hi;
                if (r.omega > 0.0f) r.omega = -r.bounce * r.omega;
                if (!r.reachedHi) {
                    r.reachedHi = true;
                    events.rotorDown = true;
                    ++stats.chain;
                    if (state == State::Running) magneticEvent();
                    if (r.spendOnHi) {
                        charge[r.body->id] = 0.0f;
                        for (Attached& at : r.attached) charge[at.body->id] = 0.0f;
                    }
                }
            }
            if (r.angle <= r.lo) {
                r.angle = r.lo;
                if (r.omega < 0.0f) r.omega = -r.bounce * r.omega;
            }
            if (w0 * r.omega < 0.0f) r.peak = std::fabs(r.angle);   // 折り返した
        }
        const Quat R = Quat::fromAxisAngle(r.axis, r.angle);
        const Vec3 w = r.axis * r.omega;
        auto place = [&](RigidBody* b, const Vec3& pos0, const Quat& rot0) {
            const Vec3 rel = R.rotate(pos0 - r.pivot);
            b->position    = r.pivot + rel;
            b->orientation = R * rot0;
            b->kinematicVelocity        = cross(w, rel);
            b->kinematicAngularVelocity = w;
        };
        place(r.body, r.basePos, r.baseRot);
        for (Attached& at : r.attached) place(at.body, at.closedPos, at.closedRot);
    }
}

// 磁化する鋼: プレイヤーが近いと、触れた側がプレイヤーと異極になる向きへ（rate の速さで）磁化し、
// 離れると decay 秒で消える。磁化は物どうしの双極子（永久モーメント）にも反映する
void FlowWorld::updateSteel(float dt) {
    for (Steel& s : steels) {
        Vec3  r;
        float d;
        surfaceTo(s.body, player->position, r, d);
        d -= player->shape.radius;
        Vec3 m = s.mLocal;
        if (state == State::Running && d < s.range) {
            const Vec3  target = s.body->orientation.toMat3().transposed() * (r * (float)(-polarity));
            const Vec3  diff   = target - m;
            const float L = length(diff), stepL = s.rate * dt;
            m = L <= stepL ? target : m + diff * (stepL / L);
        } else {
            const float L = length(m);
            if (L > 0.0f) m = m * (std::max(0.0f, L - dt / s.decay) / L);
        }
        s.mLocal = m;
        if (s.magnetIndex >= 0) world.magnets().bodies()[s.magnetIndex].permanentLocal = m * s.moment;
        const float L = length(m);
        if (!s.announced && L > 0.5f) {
            s.announced = true;
            events.magnetized = true;
            if (state == State::Running) magneticEvent();
        }
        if (s.announced && L < 0.2f) s.announced = false;
    }
}

// 磁気振り子: 綱がつながる・張る・切れる（サブステップごと。world.step の後）
void FlowWorld::updateSwings(float h) {
    int active = -1;
    for (int i = 0; i < (int)swings.size(); ++i) {
        Swing& s = swings[i];
        const bool attract = polarity * s.pole < 0;
        const Vec3  r = player->position - s.body->position;
        const float L = length(r);
        if (!attract || state != State::Running || active >= 0) {
            s.engaged = false;
            if (!attract) s.snapped = false;
            continue;
        }
        if (s.snapped) {   // 切れたあとは、離れるまでつながらない
            if (L > s.length + 1.0f) s.snapped = false;
            continue;
        }
        if (!s.engaged && L <= s.length && dot(player->velocity, r) >= 0.0f) {   // いちばん近づいた（離れ始めた）
            s.engaged = true;
            s.rod     = std::max(L, s.minLength);
            events.swing = true;
            if (!used[s.body->id]) { used[s.body->id] = 1; ++stats.magnetsUsed; }
            magneticEvent();
        }
        if (!s.engaged) continue;
        active = i;
        if (L < 1e-4f) continue;
        // 磁力で錨から一定の距離に保つ（たるまない綱）。外へ引っ張られたぶんを引き戻すのに要る撃力が、
        // 支えられる力 maxHold（重さの倍数）x h を超えたら、引き戻しきれずに綱が伸びる（滑る）。
        // 綱がつながる距離 length を超えたら切れて、振り飛ばされる
        const Vec3  n  = r / L;
        const float vr = dot(player->velocity, n);
        const float maxDv = s.maxHold * G_ACC * h;
        if (L > s.rod && vr > maxDv) {
            player->velocity -= n * maxDv;
            s.rod = L;
            if (s.rod > s.length) {
                s.engaged = false;
                s.snapped = true;
                events.snapped = true;
                active = -1;
            }
            continue;
        }
        player->position = s.body->position + n * s.rod;
        player->velocity -= n * vr;
    }
    // 錨のまわりを回った角度（振れる面の中で）
    auto angleOf = [&](const Swing& s) {
        const Vec3 r  = player->position - s.body->position;
        const Vec3 n  = lengthSq(s.planeN) > 0.0f ? s.planeN : Vec3{0, 0, 1};
        Vec3 e1 = cross(Vec3{0, 1, 0}, n);
        if (lengthSq(e1) < 1e-6f) e1 = Vec3{1, 0, 0};
        e1 = normalize(e1);
        const Vec3 e2 = cross(n, e1);
        return std::atan2(dot(r, e2), dot(r, e1));
    };
    if (active != swingActive) {
        swingActive = active;
        swingAngle  = 0.0f;
        if (active >= 0) swingPrev = angleOf(swings[active]);
        return;
    }
    if (active < 0) return;
    const float a = angleOf(swings[active]);
    float da = a - swingPrev;
    if (da >  PHYS_PI) da -= 2.0f * PHYS_PI;
    if (da < -PHYS_PI) da += 2.0f * PHYS_PI;
    swingPrev = a;
    const float before = swingAngle;
    swingAngle += da;
    if (std::floor(std::fabs(swingAngle) / (2.0f * PHYS_PI)) > std::floor(std::fabs(before) / (2.0f * PHYS_PI)))
        events.lap = true;
}

// 発射台: 座っているか、チェックポイントに着いたか
void FlowWorld::updatePads(float dt) {
    seatedPad = -1;
    for (int i = 0; i < (int)pads.size(); ++i) {
        Pad& p = pads[i];
        if (p.cooldown > 0.0f) p.cooldown = std::max(0.0f, p.cooldown - dt);
        if (p.cooldown <= 0.0f && length(player->position - p.seat) < p.seatRadius) seatedPad = i;
    }
    if (state != State::Running || seatedPad < 0) { lastSeat = -1; return; }
    const Pad& p = pads[seatedPad];
    if (seatedPad != lastSeat && p.hold != 0 && polarity != p.hold) {   // 座ったら、台が決めた極性にする
        polarity = p.hold;
        events.switched = true;
    }
    lastSeat = seatedPad;
    if (p.checkpoint > checkpoint) {
        checkpoint = p.checkpoint;
        while ((int)splits.size() <= checkpoint) splits.push_back(stats.time);
        if (p.zone >= 0 && p.zone < (int)zoneStart.size()) zoneStart[p.zone] = stats.time;   // 区間の時計を始める
        if (p.zone >= 0 && p.zone != zone) { zone = p.zone; events.zoneEntered = true; }    // 着いた発射台の区間へ
        events.checkpoint = true;
    }
}

// プレイヤーのいる区間（今の区間の箱の中にいるあいだは変えない）。カメラの向きと落下の高さを写す
void FlowWorld::updateZone() {
    if (zones.empty() || !player) return;
    const Vec3 p = player->position;
    if (zone < 0 || zone >= (int)zones.size() || !insideBox(p, zones[zone].lo, zones[zone].hi)) {
        for (int i = 0; i < (int)zones.size(); ++i) {
            if (!insideBox(p, zones[i].lo, zones[i].hi)) continue;
            if (i != zone && state == State::Running) events.zoneEntered = true;
            zone = i;
            break;
        }
    }
    if (zone < 0) return;
    const Zone& z = zones[zone];
    killY      = z.killY;
    viewDir    = z.viewDir;
    overview   = z.overview;
    viewCenter = z.viewCenter;
    viewDist   = z.viewDist;
}

float FlowWorld::zoneTime(int z) const {
    if (z < 0 || z >= (int)zoneStart.size()) return stats.time;
    return zoneStart[z] >= 0.0f ? stats.time - zoneStart[z] : 0.0f;
}

// サイクロトロン: 隙間を通り抜けたか。抜けて次の半分に入りきったとき、極が合っていれば進む向きに押す
void FlowWorld::updateCyclotrons(float dt) {
    for (Cyclotron& c : cyclotrons) {
        c.flash = std::max(0.0f, c.flash - 2.5f * dt);
        Vec3  rIn;
        float h;
        if (!cyclotronInside(c, player->position, &rIn, &h)) {
            c.inside = false;
            c.side   = 0;
            continue;
        }
        const float u = dot(rIn, c.gapN);
        int s = c.side;
        if (std::fabs(u) >= c.gapHalf) s = u > 0.0f ? 1 : -1;   // 隙間の中では前の半分のまま
        if (c.inside && c.side != 0 && s != c.side) c.turns += 0.5f;
        if (!c.inside) c.kickedSide = s;   // 入ったときにいた半分では押さない
        c.inside = true;
        c.side   = s;
        if (s != 0 && s != c.kickedSide && std::fabs(u) >= c.gapHalf + c.fringe) {   // 入りきった
            c.kickedSide = s;
            if (polarity * s > 0 && state == State::Running) {
                const Vec3  v  = player->velocity - c.axis * dot(player->velocity, c.axis);
                const float vl = length(v);
                if (vl > 1e-3f) player->velocity += v * (c.kick / vl);
                ++c.boosts;
                c.flash = 1.0f;
                events.boost = true;
                magneticEvent();
            }
        }
    }
}

// ガウス加速リング: 効く範囲にいる輪を点け、前へ抜けたときに入ったときより速ければ「押された」
void FlowWorld::updateRings(float dt) {
    const Vec3  p     = player->position;
    const float speed = length(player->velocity);
    for (GaussRing& g : rings) {
        g.flash = std::max(0.0f, g.flash - 2.5f * dt);
        const Vec3  d  = p - g.center;
        const float x  = dot(d, g.axis);
        const bool  in = std::fabs(x) < g.reach && lengthSq(d - g.axis * x) < g.radius * g.radius;
        if (in && !g.lit) g.entrySpeed = speed;
        if (!in && g.lit && x > 0.0f) {   // 前へ抜けた
            g.boosted = speed > g.entrySpeed + 0.1f;
            if (g.boosted && state == State::Running) {
                g.flash = 1.0f;
                events.boost = true;
                magneticEvent();
            }
        }
        g.lit = in;
    }
}

// プレイヤーを引いている軌道コアのまわりを回った角度（回転の向きを + にする）
void FlowWorld::updateOrbit() {
    const Vec3 p = player->position;
    int   best  = -1;
    float bestD = 1e9f;
    for (int i = 0; i < (int)cores.size(); ++i) {
        const Core& c = cores[i];
        if (c.plain) continue;
        const float d = length(p - c.body->position) - c.body->shape.radius - player->shape.radius;
        if (d < c.reach && polarity * c.pole < 0 && d < bestD) { best = i; bestD = d; }
    }
    auto angleAround = [&](const Core& c) {
        Vec3 e1, e2;
        buildTangents(c.axis, e1, e2);
        if (dot(cross(e1, e2), c.axis) < 0.0f) std::swap(e1, e2);
        const Vec3 r = p - c.body->position;
        return std::atan2(dot(r, e2), dot(r, e1));
    };
    if (best != orbitCore) {
        orbitCore  = best;
        orbitAngle = 0.0f;
        if (best >= 0) orbitPrev = angleAround(cores[best]);
        return;
    }
    if (best < 0) return;
    const float a = angleAround(cores[best]);
    float da = a - orbitPrev;
    if (da >  PHYS_PI) da -= 2.0f * PHYS_PI;
    if (da < -PHYS_PI) da += 2.0f * PHYS_PI;
    orbitPrev = a;
    const float before = orbitAngle;
    orbitAngle += da;
    if (da > 0.0f) stats.laps += da / (2.0f * PHYS_PI);
    if (std::floor(orbitAngle / (2.0f * PHYS_PI)) > std::floor(before / (2.0f * PHYS_PI)) && orbitAngle > 0.0f)
        events.lap = true;
}

void FlowWorld::step(float dt) {
    if (!player) return;
    for (Core& c : cores) c.spin += dt * 1.6f;
    for (Swing& s : swings) s.spin += dt;
    if (state == State::Ready) {   // 開始するまで止めておく（開始の瞬間の状態を毎回同じにする）
        float s = 1.0f;
        netForce = computePlayerForce(&links, &s) * s;
        for (PlayerLink& l : links) l.force = l.force * s;
        vortexForce = Vec3{0, 0, 0};
        return;
    }
    world.gravity  = Vec3{0, -G_ACC * params.gravity, 0};
    world.substeps = 1;
    const int   n = std::max(1, params.substeps);
    const float h = dt / (float)n;
    const bool  running = state == State::Running;
    for (int k = 0; k < n; ++k) {
        if (!animators.empty()) updateAnimators((float)k * h);
        applyPlayerForce();
        if (!rotors.empty()) updateRotors(h);
        world.step(h);
        if (!swings.empty()) updateSwings(h);
        if (running) {   // 発射台の上では、速さを磁気ブレーキで落として止める
            for (const Pad& p : pads)
                if (p.cooldown <= 0.0f && lengthSq(p.launch) > 0.0f && length(player->position - p.seat) < p.seatRadius)
                    player->velocity = player->velocity * (1.0f / (1.0f + 15.0f * h));
        }
        const float v = length(player->velocity);
        if (v > params.speedCap) player->velocity = player->velocity * (params.speedCap / v);
    }

    const float speed = length(player->velocity);
    if (running) {
        stats.time += dt;
        stats.maxSpeed = std::max(stats.maxSpeed, speed);
        comboClock += dt;
        if (comboClock > COMBO_GAP) stats.combo = 0;
    }
    if (prevSpeed - speed > 4.0f) events.hit = true;   // 何かに強く当たった
    prevSpeed = speed;

    // 使った磁石と「吸着」: いちばん強く引いている磁性体が、重さの 1.5 倍以上で別の物に替わった
    const float weight = G_ACC / player->invMass;
    int   dominant = -1;
    float domF = 0.0f;
    for (const PlayerLink& l : links) {
        const float f = length(l.force);
        if (running && isMagnet(kinds[l.body]) && f >= weight && !used[l.body]) {
            used[l.body] = 1;
            ++stats.magnetsUsed;
        }
        if (l.attract && f > domF) { domF = f; dominant = l.body; }
    }
    if (running && dominant >= 0 && domF >= 1.5f * weight && dominant != lastCapture) {
        lastCapture = dominant;
        events.capture = true;
        magneticEvent();
    }
    if (running) updateOrbit();
    if (!cyclotrons.empty()) updateCyclotrons(dt);
    if (!rings.empty()) updateRings(dt);

    // 連鎖: 動く物が初めて 1 m/s を超えた
    for (const auto& bp : world.getBodies()) {
        const RigidBody* b = bp.get();
        const Kind       k = kindOf(b);
        if ((k != Kind::MovableMagnet && k != Kind::Iron) || moved[b->id] || b->isStatic()) continue;
        if (uniformPole[b->id] != 0) continue;   // 鎖の球は連鎖に数えない（揺れるだけ）
        if (length(b->velocity) > 1.0f) {
            moved[b->id] = 1;
            flash[b->id] = 1.0f;
            ++stats.chain;
            events.chain = true;
            if (running) magneticEvent();
        }
    }

    updateGates(dt);
    if (!steels.empty()) updateSteel(dt);
    if (!pads.empty()) updatePads(dt);
    updateZone();

    if (running) {
        if (hasGoal && insideBox(player->position, goalLo, goalHi)) {
            state = State::Cleared;
            events.cleared = true;
        } else if (player->position.y < killY) {
            state = State::Failed;
            events.failed = true;
            failTimer = 0.0f;
        }
        ++stepCount;
        if (stepCount % TRAIL_STEP == 0 && trail.size() < 20000) trail.push_back(player->position);
    } else if (state == State::Failed) {
        failTimer += dt;
        if (bigMap() && checkpoint >= 0 && failTimer >= RESPAWN_DELAY) {   // 大きなマップ: チェックポイントに戻る
            ++stats.falls;
            respawn();
        }
    }
}

} // namespace flow
