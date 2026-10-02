#pragma once
// ---------------------------------------------------------------------------
// electric.h : 電荷・電場・ローレンツ力・渦電流
//
//   単位系は k = 1 / (4 pi eps0) = 1。磁気の K = mu0 / 4pi = 1（magnet.h）と合わせると、
//   ガウス単位系と同じ形になる:
//       点電荷の場  E = q r / r^3          双極子の場は磁気と同じ式（dipoleField）
//       力          F = q (E + kappa v x B)
//   kappa は「この世界の 1/c」。本物の値（1 / 光速）では、手で持てる大きさの物にローレンツ力は
//   ほとんど効かないので、シーンで大きくする（光速を遅くした世界）。
//   動く電荷が作る磁場（ビオ・サバール）は入れない。磁石には電荷からの力が返らない。
//
//   静電気（applyForces の前半）
//     - 帯電した球: 中心の点電荷
//     - 導体の球: 電場で分極する（誘導双極子 p = R^3 E。鉄の m = alpha B と同じ形。前回の E から決める）
//     - 力 F = qE + (p . grad) E を全ペアで求め、作用・反作用で加える（近距離は磁石と同じクランプ）
//     - 平行板のコンデンサ: 2 枚の静的な板にはさまれた箱の中だけ、一様な電場 (V_A - V_B) / d。外では 0
//     - 一様な外部電場 externalField
//   接触での電荷の移動（exchangeCharges。接触を求めた後）
//     - 導体の球どうし: 合計の電荷を容量（球は R）の比で分け合う
//     - 導体の球とコンデンサの板: 球の電荷が R V になる（電荷はその板から出入りする）
//   ローレンツ力の磁場の部分（rotateLorentz。力を積分した後）
//     - 速度を B の軸のまわりに、角度 kappa q |B| h / m だけ厳密に回す（速さが変わらない）
//   渦電流（applyForces の後半。局所的なオームの法則）
//     - 導電率 sigma の物体の中の点ごとに、電流 J = sigma sum_i (v_点 - u_i) x B_i
//       （B_i は磁性体 i の場、u_i はその重心の速度）、力 f = J x B dV を導体に、
//       反作用 -J x B_i dV を磁性体 i の重心に加える（運動量は保存する）
//     - 表面に溜まる電荷は無視する（切れ込みでブレーキが弱まる現象や、表皮効果は出ない）
//     - 1 サブステップで相対的な動きを消しすぎないように、力に上限を付ける（安定化）
//
//   World が ElectricSystem を 1 つ持つ（World::createChargedSphere / createConductorBox / createCapacitor）。
// ---------------------------------------------------------------------------
#include "phys/magnet.h"

#include <vector>

namespace phys {

class CurrentSystem;   // 電流の導線（current.h）。磁場の源として渡す

constexpr float LIGHT_SPEED = 2.99792458e8f;   // [m/s]

// 強さの補助関数
//   接触した同じ 2 球のクーロン力が、重さの gamma 倍になる電荷: q = sqrt(gamma M g) d
float chargeFromGamma(float gamma, float mass, float contactDist, float g);
//   薄い管（半径 a、厚さ t、導電率 sigma）の中を軸に沿って落ちる双極子 m の抵抗 F = C v、
//   C = (45 pi^2 / 64) m^2 sigma t / a^4。終端速度は M g / C
float tubeDragCoefficient(float moment, float tubeRadius, float wall, float sigma);
float sigmaForTubeTerminalSpeed(float moment, float mass, float g, float tubeRadius, float wall, float vTerminal);

// 帯電・導電する物体
struct ElectricBody {
    RigidBody* body = nullptr;
    float charge    = 0.0f;
    bool  conductor = false;   // 導体の球（分極し、触れた導体・板と電荷をやり取りする）
    float alpha     = 0.0f;    // 分極率（導体の球は R^3）
    float radius    = 0.0f;    // 近距離のクランプと容量（球の半径。箱は 0）
    float sigma     = 0.0f;    // 導電率（渦電流）。0 なら流れない
    std::vector<Vec3> eddyPoints;   // 渦電流を求める点（ローカル）
    float pointVolume = 0.0f;       // 1 点の体積

    Vec3 p{0, 0, 0};           // 誘導双極子（ワールド）
    Vec3 E{0, 0, 0};           // この位置の電場（最後の評価。外部・板の場を含む）
};

// 平行板のコンデンサ: 2 枚の静的な板と、その間の箱（中だけ一様な電場）
struct Capacitor {
    RigidBody* plateA = nullptr;
    RigidBody* plateB = nullptr;
    float potentialA = 0.0f, potentialB = 0.0f;
    Vec3  center{0, 0, 0};
    Quat  orientation{};       // ローカル +y が A → B の向き
    Vec3  halfExtents{0, 0, 0};   // 板の間の箱（y がすき間の半分）
    Vec3  E{0, 0, 0};          // 箱の中の電場 (V_A - V_B) / d
    float exchanged = 0.0f;    // 板から出た電荷の累計（電荷の保存の確認用）
};

class ElectricSystem {
public:
    float kappa = 1.0f / LIGHT_SPEED;   // ローレンツ力の係数（この世界の 1/c）
    Vec3  externalField{0, 0, 0};       // 一様な外部電場
    float minDistScale = 0.9f;          // 近距離クランプ: r_eff = max(r, minDistScale * (R1 + R2))
    float eddyStability = 0.5f;         // 渦電流の力が 1 サブステップで消せる相対的な動きの割合の上限

    // 物体を登録する。charge: 電荷、conductor: 導体の球か、sigma: 導電率（渦電流）、
    // spacing: 渦電流の点の間隔（0 なら大きさから決める）。戻り値は bodies() の添字
    int  addBody(RigidBody* body, float charge, bool conductor, float sigma = 0.0f, float spacing = 0.0f);
    int  addCapacitor(RigidBody* plateA, RigidBody* plateB, float potentialA, float potentialB, const Vec3& center,
                      const Quat& orientation, const Vec3& halfExtents);
    void clear();
    bool empty() const { return bodies_.empty() && capacitors_.empty(); }
    int  indexOf(const RigidBody* body) const;

    // 各サブステップの先頭（磁力の後）: 静電気の力と渦電流の力を body に加算する。h は安定化に使う。
    // currents を渡すと、電流の導線も磁場の源になる（渦電流の反作用は導線の物体へ）
    void applyForces(const MagnetSystem& magnets, float h, const CurrentSystem* currents = nullptr);
    // 力を積分した後: 電荷を持つ動く物体の速度を、磁場のまわりに回す
    void rotateLorentz(const MagnetSystem& magnets, float h, const CurrentSystem* currents = nullptr);
    // 接触を求めた後: 触れた導体どうし・導体と板で電荷をやり取りする
    void exchangeCharges(const std::vector<Manifold>& manifolds);
    // 誘導双極子を、今の位置で iterations 回求め直す（作った直後など）
    void updateDipoles(int iterations);

    Vec3  fieldAt(const Vec3& p) const;      // 任意の点の電場
    float potentialAt(const Vec3& p) const;  // 任意の点の電位（点電荷・双極子・外部。板の場は箱の中だけ）
    float potentialEnergy() const;           // 電荷どうし・板と外部の場の中の電荷・誘導双極子
    float totalCharge() const;               // 物体の電荷の和
    float plateExchange() const;             // 板から出た電荷の累計（和は totalCharge の変化と等しい）
    // 磁性体の点のモーメント（と外部磁場、電流の導線）から B
    static Vec3 magneticFieldAt(const MagnetSystem& magnets, const Vec3& p, int skipBody = -1,
                                const CurrentSystem* currents = nullptr);

    // 渦電流の点（描画用。最後の applyForces の値）
    struct EddySample { Vec3 x, J; };
    const std::vector<EddySample>& eddySamples() const { return eddySamples_; }

    const std::vector<ElectricBody>& bodies() const { return bodies_; }
    std::vector<ElectricBody>&       bodies()       { return bodies_; }
    const std::vector<Capacitor>&    capacitors() const { return capacitors_; }
    std::vector<Capacitor>&          capacitors()       { return capacitors_; }

private:
    Vec3 capacitorField(const Vec3& p) const;      // 板の間の箱の中の電場の和
    void electrostatics(bool withForces);           // 電場を求め（と力を加え）、誘導双極子を決め直す
    void eddyCurrents(const MagnetSystem& magnets, float h, const CurrentSystem* currents);

    std::vector<ElectricBody> bodies_;
    std::vector<Capacitor>    capacitors_;
    std::vector<int>          indexById_;   // body->id → bodies_ の添字（-1 = 登録なし）
    std::vector<int>          capOfPlate_;  // body->id → capacitors_ の添字 * 2 + 板（0: A、1: B）、-1 = 板でない
    bool initialized_ = false;
    std::vector<EddySample>   eddySamples_;
};

} // namespace phys
