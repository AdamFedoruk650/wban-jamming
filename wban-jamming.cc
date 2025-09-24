#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wban-module.h"

#include <algorithm>
#include <filesystem>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <unordered_map>
#include <system_error>

using namespace ns3;
using namespace ns3::wban;

namespace
{
namespace fs = std::filesystem;

const fs::path&
GetWbanRootDir()
{
    static const fs::path kRoot = fs::absolute(fs::path(__FILE__)).parent_path().parent_path();
    return kRoot;
}

fs::path
ResolveCsvPath(const std::string& requested)
{
    if (requested.empty())
    {
        return {};
    }

    fs::path requestedPath(requested);
    if (requestedPath.is_absolute())
    {
        return requestedPath;
    }

    static const fs::path kDefaultCsvDir = GetWbanRootDir() / "output" / "scan";
    return kDefaultCsvDir / requestedPath;
}

std::string
ToLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
    return value;
}

BodyOrganOption
ParseBodyOrganOption(const std::string& organ)
{
    static const std::unordered_map<std::string, BodyOrganOption> kOrganMap = {
        {"heart", BodyOrganOption::HEART_402_MHZ},
        {"heart-402", BodyOrganOption::HEART_402_MHZ},
        {"heart-2400", BodyOrganOption::HEART_2400_MHZ},
        {"small-intestine-402", BodyOrganOption::SMALL_INTESTINE_402_MHZ},
        {"small-intestine-2400", BodyOrganOption::SMALL_INTESTINE_2400_MHZ},
        {"small-intestine-916.5", BodyOrganOption::SMALL_INTESTINE_916_5_MHZ},
        {"large-intestine-2400", BodyOrganOption::LARGE_INTESTINE_2400_MHZ},
        {"fat-402", BodyOrganOption::FAT_402_MHZ},
        {"fat-2400", BodyOrganOption::FAT_2400_MHZ},
        {"skin-402", BodyOrganOption::SKIN_402_MHZ},
        {"skin-863", BodyOrganOption::SKIN_863_MHZ},
        {"skin-2400", BodyOrganOption::SKIN_2400_MHZ},
        {"kidney-402", BodyOrganOption::KIDNEY_402_MHZ},
        {"kidney-2400", BodyOrganOption::KIDNEY_2400_MHZ},
    };

    auto key = ToLower(organ);
    auto it = kOrganMap.find(key);
    if (it != kOrganMap.end())
    {
        return it->second;
    }

    std::cout << "[WBAN] Nieznana opcja organu '" << organ
              << "'. Używam domyślnej konfiguracji heart-402." << std::endl;
    return BodyOrganOption::HEART_402_MHZ;
}

std::string
BodyOrganOptionToString(BodyOrganOption option)
{
    switch (option)
    {
    case BodyOrganOption::SMALL_INTESTINE_2400_MHZ:
        return "small-intestine-2400";
    case BodyOrganOption::SMALL_INTESTINE_916_5_MHZ:
        return "small-intestine-916.5";
    case BodyOrganOption::FAT_2400_MHZ:
        return "fat-2400";
    case BodyOrganOption::FAT_402_MHZ:
        return "fat-402";
    case BodyOrganOption::SKIN_2400_MHZ:
        return "skin-2400";
    case BodyOrganOption::SKIN_863_MHZ:
        return "skin-863";
    case BodyOrganOption::SKIN_402_MHZ:
        return "skin-402";
    case BodyOrganOption::LARGE_INTESTINE_2400_MHZ:
        return "large-intestine-2400";
    case BodyOrganOption::SMALL_INTESTINE_402_MHZ:
        return "small-intestine-402";
    case BodyOrganOption::HEART_2400_MHZ:
        return "heart-2400";
    case BodyOrganOption::KIDNEY_2400_MHZ:
        return "kidney-2400";
    case BodyOrganOption::HEART_402_MHZ:
        return "heart-402";
    case BodyOrganOption::KIDNEY_402_MHZ:
        return "kidney-402";
    }
    return "unknown";
}

} // namespace

//m_phyOption = WbanPhyOption::NB_402_MHZ_75_9;

struct SimulationConfig
{
    double txX;
    double txY;
    double rxX;
    double rxY;
    double jamX;
    double jamY;
    BodyOrganOption organOption;
};

struct SimulationResult
{
    uint32_t noJamSent = 0;
    uint32_t noJamRx = 0;
    uint32_t jamSentTx = 0;
    uint32_t jamRxTx = 0;
    uint32_t jamSentJam = 0;
    uint32_t jamRxJam = 0;
    double bodyRxPowerDbm = 0.0;
    double bodyLossDb = 0.0;
    double jamRxPowerDbm = 0.0;
    double jamLossDb = 0.0;
    BodyDielectricParameters params;
    double txX = 0.0;
    double txY = 0.0;
    double rxX = 0.0;
    double rxY = 0.0;
    double jamX = 0.0;
    double jamY = 0.0;
};

struct SimulationContext
{
    Ptr<Node> txNode;
    Ptr<Node> rxNode;
    Ptr<Node> jamNode;
    Ptr<WbanNetDevice> txDev;
    Ptr<WbanNetDevice> rxDev;
    Ptr<WbanNetDevice> jamDev;
    Ptr<BodyPropagationLossModel> bodyLoss;
    Ptr<LogDistancePropagationLossModel> pathLoss;
    Ptr<SingleModelSpectrumChannel> channel;
    Ptr<ConstantPositionMobilityModel> mTx;
    Ptr<ConstantPositionMobilityModel> mRx;
    Ptr<ConstantPositionMobilityModel> mJam;
};

void ResetCounters();
void RxIndication(uint32_t psduLength, Ptr<Packet> p, uint8_t packetSize);
SimulationContext CreateSimulationContext(BodyOrganOption organ);
SimulationResult RunScenario(SimulationContext& ctx, const SimulationConfig& config, bool enableLogs);

// ===== Parametry eksperymentu =====z
static uint32_t kNoJamPackets   = 5000;   // ile pakietów bez jammingu (można zmienić z CLI)
static uint32_t kWithJamPackets = 5000;   // ile pakietów z jammingiem (można zmienić z CLI)
static const double kPktGapSeconds    = 0.02; //kPktGapSeconds dla różnych obciążeń (np. 0.1s, 0.02s, 0.005s).
static const double kGapBetweenPhases = 1.0;
static const uint32_t kPrintEvery     = 500;

// WBAN / PHY
static const uint32_t kChannelNumber = 1;
static const int      kPayloadBytes  = 32;
static const double   kTxPowerDbm    = -16.0; //dobrze -20, -16, -10, 0
static const double   kJamBoostDb    = 0.0; // +6, +10, +20
static const double   kRxSensitivity = -98; // -113.97

// ===== Liczniki =====
static uint32_t g_noJamSent = 0, g_noJamRx = 0;
static uint32_t g_jamSentTx = 0, g_jamRxTx = 0;
static uint32_t g_jamSentJam = 0, g_jamRxJam = 0;
static bool     g_jammingActive = false;

void
ResetCounters()
{
    g_noJamSent = 0;
    g_noJamRx = 0;
    g_jamSentTx = 0;
    g_jamRxTx = 0;
    g_jamSentJam = 0;
    g_jamRxJam = 0;
    g_jammingActive = false;
}

SimulationContext
CreateSimulationContext(BodyOrganOption organ)
{
    SimulationContext ctx;
    ctx.txNode = CreateObject<Node>();
    ctx.rxNode = CreateObject<Node>();
    ctx.jamNode = CreateObject<Node>();

    ctx.txDev = CreateObject<WbanNetDevice>();
    ctx.rxDev = CreateObject<WbanNetDevice>();
    ctx.jamDev = CreateObject<WbanNetDevice>();

    ctx.channel = CreateObject<SingleModelSpectrumChannel>();
    ctx.bodyLoss = CreateObject<BodyPropagationLossModel>();
    ctx.pathLoss = CreateObject<LogDistancePropagationLossModel>();
    ctx.bodyLoss->SetBodyOptions(organ);
    ctx.channel->AddPropagationLossModel(ctx.bodyLoss);
    ctx.channel->AddPropagationLossModel(ctx.pathLoss);

    ctx.txDev->SetChannel(ctx.channel);
    ctx.rxDev->SetChannel(ctx.channel);
    ctx.jamDev->SetChannel(ctx.channel);

    ctx.txNode->AddDevice(ctx.txDev);
    ctx.rxNode->AddDevice(ctx.rxDev);
    ctx.jamNode->AddDevice(ctx.jamDev);

    ctx.mTx = CreateObject<ConstantPositionMobilityModel>();
    ctx.mRx = CreateObject<ConstantPositionMobilityModel>();
    ctx.mJam = CreateObject<ConstantPositionMobilityModel>();
    ctx.txDev->GetPhy()->SetMobility(ctx.mTx);
    ctx.rxDev->GetPhy()->SetMobility(ctx.mRx);
    ctx.jamDev->GetPhy()->SetMobility(ctx.mJam);

    ctx.bodyLoss->ClearBodyMobility();
    ctx.bodyLoss->AddBodyMobility(ctx.mTx);

    WbanSpectrumValueHelper svh;
    Ptr<SpectrumValue> psdTx = svh.CreateTxPowerSpectralDensity(kTxPowerDbm, kChannelNumber);
    Ptr<SpectrumValue> psdJam = svh.CreateTxPowerSpectralDensity(kTxPowerDbm + kJamBoostDb, kChannelNumber);
    ctx.txDev->GetPhy()->SetTxPowerSpectralDensity(psdTx);
    ctx.jamDev->GetPhy()->SetTxPowerSpectralDensity(psdJam);

    ctx.rxDev->GetPhy()->SetRxSensitivity(kRxSensitivity);
    ctx.rxDev->GetPhy()->SetPhyDataIndicationCallback(MakeCallback(&RxIndication));

    return ctx;
}

// ===== Tag źródła =====
class SrcTag : public Tag
{
public:
    enum SrcType { TX = 1, JAM = 2 };
    SrcTag(SrcType t = TX) : m_type(t) {}

    static TypeId GetTypeId (void) {
        static TypeId tid = TypeId("SrcTag")
            .SetParent<Tag>()
            .AddConstructor<SrcTag>();
        return tid;
    }
    TypeId GetInstanceTypeId (void) const override { return GetTypeId(); }
    uint32_t GetSerializedSize (void) const override { return 1; }
    void Serialize (TagBuffer i) const override { i.WriteU8((uint8_t)m_type); }
    void Deserialize (TagBuffer i) override { m_type = (SrcType) i.ReadU8(); }
    void Print (std::ostream &os) const override { os << "src=" << (m_type==TX?"TX":"JAM"); }

    void Set(SrcType t) { m_type = t; }
    SrcType Get() const { return m_type; }

private:
    SrcType m_type;
};

void RxIndication(uint32_t psduLength, Ptr<Packet> p, uint8_t packetSize)
{
    SrcTag tag;
    if (p->PeekPacketTag(tag)) {
        if (g_jammingActive) {
            if (tag.Get() == SrcTag::TX) g_jamRxTx++;
            else g_jamRxJam++;
        } else {
            g_noJamRx++;
        }
    }
}
// t   r       j

SimulationResult
RunScenario(SimulationContext& ctx, const SimulationConfig& config, bool enableLogs)
{
    ResetCounters();
    ctx.bodyLoss->SetBodyOptions(config.organOption);
    ctx.mTx->SetPosition(Vector(config.txX, config.txY, 0));
    ctx.mRx->SetPosition(Vector(config.rxX, config.rxY, 0));
    ctx.mJam->SetPosition(Vector(config.jamX, config.jamY, 0));

    double bodyRxPowerDbm = ctx.bodyLoss->CalcRxPower(kTxPowerDbm, ctx.mTx, ctx.mRx);
    double bodyLossDb = kTxPowerDbm - bodyRxPowerDbm;
    double jamRxPowerDbm = ctx.pathLoss->CalcRxPower(kTxPowerDbm + kJamBoostDb, ctx.mJam, ctx.mRx);
    double jamLossDb = (kTxPowerDbm + kJamBoostDb) - jamRxPowerDbm;
    BodyDielectricParameters params = ctx.bodyLoss->m_parameters;

    if (enableLogs)
    {
        std::cout << "[BodyPropagationLossModel] organ="
                  << BodyOrganOptionToString(config.organOption)
                  << " DoCalcRxPower(txPowerDbm=" << kTxPowerDbm
                  << ", txPos=" << ctx.mTx->GetPosition()
                  << ", rxPos=" << ctx.mRx->GetPosition()
                  << ") = " << bodyRxPowerDbm << " dBm" << std::endl;
        std::cout << "    -> attenuation due to body = " << bodyLossDb << " dB" << std::endl;
        std::cout << "    -> jammer path rxPower = " << jamRxPowerDbm << " dBm" << std::endl;
        std::cout << "    -> jammer path loss = " << jamLossDb << " dB" << std::endl;
        std::cout << "    -> dielectric params: organConductivity=" << params.organConductivity
                  << ", organPermittivity=" << params.organPermittivity
                  << ", skinConductivity=" << params.skinConductivity
                  << ", skinPermittivity=" << params.skinPermittivity << std::endl;
    }

    const bool logProgress = enableLogs;

    Simulator::Schedule(Seconds(0.2), [&, logProgress]() {
        if (logProgress)
        {
            std::cout << "=== FAZA 1: BEZ jammingu ===\n";
        }
        ctx.txDev->GetPhy()->PhySetTRXStateRequest(WbanPhyState::PHY_TX_ON);
        ctx.rxDev->GetPhy()->PhySetTRXStateRequest(WbanPhyState::PHY_RX_ON);
        g_jammingActive = false;
    });

    for (uint32_t i = 0; i < kNoJamPackets; ++i)
    {
        Simulator::Schedule(Seconds(0.5 + i * kPktGapSeconds), [&, i, logProgress]() {
            Ptr<Packet> p = Create<Packet>(kPayloadBytes);
            SrcTag tag(SrcTag::TX);
            p->AddPacketTag(tag);
            ctx.txDev->GetPhy()->PhyDataRequest(kPayloadBytes, p);
            ++g_noJamSent;
            if (logProgress && ((i + 1) % kPrintEvery == 0))
            {
                std::cout << "Faza 1: TX wysłano " << (i + 1)
                          << ", RX odebrał " << g_noJamRx << "\n";
            }
        });
    }

    const double phase2Start = 0.5 + kNoJamPackets * kPktGapSeconds + kGapBetweenPhases;
    const Time phase2StartTime = Seconds(phase2Start);

    Simulator::Schedule(phase2StartTime, [&, logProgress]() {
        if (logProgress)
        {
            std::cout << "\n=== FAZA 2: Z jammerem ===\n";
        }
        ctx.jamDev->GetPhy()->PhySetTRXStateRequest(WbanPhyState::PHY_TX_ON);
        g_jammingActive = true;
    });

    for (uint32_t i = 0; i < kWithJamPackets; ++i)
    {
        Simulator::Schedule(phase2StartTime + Seconds(i * kPktGapSeconds), [&, logProgress]() {
            Ptr<Packet> jp = Create<Packet>(32);
            SrcTag tag(SrcTag::JAM);
            jp->AddPacketTag(tag);
            ctx.jamDev->GetPhy()->PhyDataRequest(jp->GetSize(), jp);
            ++g_jamSentJam;
        });
    }

    for (uint32_t i = 0; i < kWithJamPackets; ++i)
    {
        Simulator::Schedule(phase2StartTime + Seconds(i * kPktGapSeconds), [&, i, logProgress]() {
            Ptr<Packet> p = Create<Packet>(kPayloadBytes);
            SrcTag tag(SrcTag::TX);
            p->AddPacketTag(tag);
            ctx.txDev->GetPhy()->PhyDataRequest(kPayloadBytes, p);
            ++g_jamSentTx;
            if (logProgress && ((i + 1) % kPrintEvery == 0))
            {
                std::cout << "Faza 2: TX wysłano " << (i + 1)
                          << ", RX odebrał TX " << g_jamRxTx
                          << ", RX odebrał JAM " << g_jamRxJam << "\n";
            }
        });
    }

    double simStop = phase2Start + kWithJamPackets * kPktGapSeconds + 1.0;
    Simulator::Stop(Seconds(simStop));
    Simulator::Run();

    if (enableLogs)
    {
        std::cout << "\n=== PODSUMOWANIE ===\n";
        std::cout << "FAZA 1 (bez jammer’a): TX wysłane " << g_noJamSent
                  << " | RX odebrał " << g_noJamRx
                  << " | stracone " << (g_noJamSent - g_noJamRx) << "\n";
        std::cout << "FAZA 2 (z jammer’em): TX wysłane " << g_jamSentTx
                  << " | RX odebrał " << g_jamRxTx
                  << " | stracone " << (g_jamSentTx - g_jamRxTx) << "\n";
        std::cout << "FAZA 2 (z jammer’em): JAM wysłane " << g_jamSentJam
                  << " | RX odebrał " << g_jamRxJam
                  << " | stracone " << (g_jamSentJam - g_jamRxJam) << "\n";
    }

    SimulationResult result;
    result.noJamSent = g_noJamSent;
    result.noJamRx = g_noJamRx;
    result.jamSentTx = g_jamSentTx;
    result.jamRxTx = g_jamRxTx;
    result.jamSentJam = g_jamSentJam;
    result.jamRxJam = g_jamRxJam;
    result.bodyRxPowerDbm = bodyRxPowerDbm;
    result.bodyLossDb = bodyLossDb;
    result.jamRxPowerDbm = jamRxPowerDbm;
    result.jamLossDb = jamLossDb;
    result.params = params;
    result.txX = config.txX;
    result.txY = config.txY;
    result.rxX = config.rxX;
    result.rxY = config.rxY;
    result.jamX = config.jamX;
    result.jamY = config.jamY;

    return result;
}

int main(int argc, char* argv[])
{
    double txX = 0.0, txY = 0.0, rxX = 0.3, rxY = 0.0, jamX = 43.0, jamY = 0.0;
    std::string bodyOrgan = "heart-402";
    std::string scanCsv;
    double scanStart = 0.1;
    double scanStop = 2.0;
    double scanStep = 0.1;
    double jamThreshold = 0.05;
    std::string scanTarget = "rx";

    CommandLine cmd;
    cmd.AddValue("txX", "Pozycja X nadajnika", txX);
    cmd.AddValue("txY", "Pozycja Y nadajnika", txY);
    cmd.AddValue("rxX", "Pozycja X odbiornika", rxX);
    cmd.AddValue("rxY", "Pozycja Y odbiornika", rxY);
    cmd.AddValue("jamX", "Pozycja X jammer’a", jamX);
    cmd.AddValue("jamY", "Pozycja Y jammer’a", jamY);
    cmd.AddValue("noJamPackets", "Liczba pakietów bez jammingu", kNoJamPackets);
    cmd.AddValue("jamPackets", "Liczba pakietów z jammer’a", kWithJamPackets);
    cmd.AddValue("bodyOrgan", "Model tłumienia dla danego organu (np. heart-402)", bodyOrgan);
    cmd.AddValue("scanCsv", "Ścieżka do pliku CSV z przebiegiem skanowania (opcjonalnie)", scanCsv);
    cmd.AddValue("scanStart", "Pozycja początkowa (m) dla skanowanego węzła", scanStart);
    cmd.AddValue("scanStop", "Pozycja końcowa (m) dla skanowanego węzła", scanStop);
    cmd.AddValue("scanStep", "Krok położenia (m) w skanowaniu CSV", scanStep);
    cmd.AddValue("jamThreshold", "Próg (0-1) klasyfikacji jammingu w fazie 2", jamThreshold);
    cmd.AddValue("scanTarget", "Który węzeł skanujemy w CSV: rx lub jam", scanTarget);
    cmd.Parse(argc, argv);

    if (scanStep <= 0.0)
    {
        std::cerr << "[CLI] scanStep musi być > 0" << std::endl;
        return 1;
    }
    jamThreshold = std::max(0.0, std::min(1.0, jamThreshold));
    std::string scanTargetKey = ToLower(scanTarget);
    bool scanJam = (scanTargetKey == "jam" || scanTargetKey == "jammer" || scanTargetKey == "j");

    BodyOrganOption organOption = ParseBodyOrganOption(bodyOrgan);

    SimulationContext ctx = CreateSimulationContext(organOption);

    SimulationConfig baseConfig{txX, txY, rxX, rxY, jamX, jamY, organOption};
    RunScenario(ctx, baseConfig, true);

    fs::path scanCsvPath = ResolveCsvPath(scanCsv);
    if (!scanCsv.empty() && scanStop >= scanStart)
    {
        SimulationConfig scanConfig = baseConfig;
        fs::path parent = scanCsvPath.parent_path();
        if (!parent.empty())
        {
            std::error_code ec;
            fs::create_directories(parent, ec);
            if (ec)
            {
                std::cerr << "[CSV] Nie można utworzyć katalogu '" << parent.string()
                          << "': " << ec.message() << std::endl;
                return 1;
            }
        }

        std::ofstream csv(scanCsvPath);
        if (!csv.is_open())
        {
            std::cerr << "[CSV] Nie można otworzyć pliku '" << scanCsvPath.string() << "' do zapisu" << std::endl;
            return 1;
        }

        csv << "rxX,rxY,txRxDistance,rxJamDistance,scanCoordinate,bodyLossDb,bodyRxPowerDbm,jamRxPowerDbm,jamLossDb,noJamSuccessRate,jamSuccessRate,isJammed,noJamPacketsRx,jamPacketsRx,jamPacketsFromJammerRx\n";

        double firstSafeDistance = std::numeric_limits<double>::quiet_NaN();
        const double epsilon = scanStep * 0.5;

        for (double scanPos = scanStart; scanPos <= scanStop + epsilon; scanPos += scanStep)
        {
            if (scanJam)
            {
                scanConfig.jamX = scanPos;
                scanConfig.rxX = baseConfig.rxX;
                scanConfig.jamY = baseConfig.jamY;
                scanConfig.rxY = baseConfig.rxY;
            }
            else
            {
                scanConfig.rxX = scanPos;
                scanConfig.rxY = baseConfig.rxY;
                scanConfig.jamX = baseConfig.jamX;
                scanConfig.jamY = baseConfig.jamY;
            }
            SimulationResult res = RunScenario(ctx, scanConfig, false);

            double scanCoordinate = scanPos;

            double noJamRate = res.noJamSent ? static_cast<double>(res.noJamRx) / res.noJamSent : 0.0;
            double jamRate = res.jamSentTx ? static_cast<double>(res.jamRxTx) / res.jamSentTx : 0.0;
            bool jammed = jamRate <= jamThreshold;
            double txRxDistance = std::hypot(res.rxX - res.txX, res.rxY - res.txY);
            double jamRxDistance = std::hypot(res.rxX - res.jamX, res.rxY - res.jamY);

            csv << res.rxX << ',' << res.rxY << ',' << txRxDistance << ',' << jamRxDistance << ','
                << scanCoordinate << ',' << res.bodyLossDb << ',' << res.bodyRxPowerDbm << ','
                << res.jamRxPowerDbm << ',' << res.jamLossDb << ','
                << noJamRate << ',' << jamRate << ',' << (jammed ? 1 : 0) << ','
                << res.noJamRx << ',' << res.jamRxTx << ',' << res.jamRxJam << '\n';

            if (!jammed && std::isnan(firstSafeDistance))
            {
                firstSafeDistance = scanJam ? jamRxDistance : txRxDistance;
            }
        }

        csv.close();
        std::cout << "[CSV] Wyniki skanowania zapisano do " << scanCsvPath.string() << std::endl;
        if (!std::isnan(firstSafeDistance))
        {
            if (scanJam)
            {
                std::cout << "[Threshold] Minimalna odległość JAM-RX dająca brak jammingu (prog="
                          << jamThreshold << ") ≈ " << firstSafeDistance << " m" << std::endl;
            }
            else
            {
                std::cout << "[Threshold] Pierwsza pozycja RX poza strefą jammingu (prog=" << jamThreshold
                          << ") dla odległości TX-RX ≈ " << firstSafeDistance << " m" << std::endl;
            }
        }
        else
        {
            std::cout << "[Threshold] W zadanym zakresie obiekt pozostaje w strefie jammingu (prog="
                      << jamThreshold << ")" << std::endl;
        }
    }

    Simulator::Destroy();
    return 0;
}
