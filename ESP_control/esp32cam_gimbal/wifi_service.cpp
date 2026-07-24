#include "wifi_service.h"

#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include <limits.h>

#include "app_config.h"


namespace
{

constexpr const char *NVS_NAMESPACE = "robot_wifi";
constexpr const char *NVS_NEXT_SLOT_KEY = "next";
constexpr size_t MAX_SSID_LENGTH = 32;
constexpr size_t MAX_PASSWORD_LENGTH = 63;


struct WiFiProfile
{
    String ssid;
    String password;
};


DNSServer dnsServer;
WebServer provisioningServer(
    AppConfig::WiFiProvisioning::WEB_PORT
);

bool restartRequested = false;
unsigned long restartRequestedAt = 0;


String slotKey(char prefix, size_t slot)
{
    char key[4];

    snprintf(
        key,
        sizeof(key),
        "%c%u",
        prefix,
        static_cast<unsigned int>(slot)
    );

    return String(key);
}


size_t loadProfiles(WiFiProfile *profiles)
{
    Preferences preferences;

    if (!preferences.begin(NVS_NAMESPACE, true))
    {
        Serial.println("Cannot open WiFi NVS namespace");
        return 0;
    }

    size_t profileCount = 0;

    for (
        size_t slot = 0;
        slot < AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS;
        ++slot
    )
    {
        const String ssidKey = slotKey('s', slot);
        const String passwordKey = slotKey('p', slot);
        const String ssid = preferences.getString(
            ssidKey.c_str(),
            ""
        );

        if (ssid.length() == 0)
        {
            continue;
        }

        profiles[profileCount].ssid = ssid;
        profiles[profileCount].password =
            preferences.getString(
                passwordKey.c_str(),
                ""
            );
        ++profileCount;
    }

    preferences.end();
    return profileCount;
}


bool saveProfile(const String &ssid, const String &password)
{
    Preferences preferences;

    if (!preferences.begin(NVS_NAMESPACE, false))
    {
        return false;
    }

    size_t targetSlot =
        AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS;
    size_t firstEmptySlot =
        AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS;

    for (
        size_t slot = 0;
        slot < AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS;
        ++slot
    )
    {
        const String key = slotKey('s', slot);
        const String savedSsid = preferences.getString(
            key.c_str(),
            ""
        );

        if (savedSsid == ssid)
        {
            targetSlot = slot;
            break;
        }

        if (
            savedSsid.length() == 0 &&
            firstEmptySlot ==
                AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
        )
        {
            firstEmptySlot = slot;
        }
    }

    if (
        targetSlot ==
        AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
    )
    {
        if (
            firstEmptySlot <
            AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
        )
        {
            targetSlot = firstEmptySlot;
        }
        else
        {
            targetSlot =
                preferences.getUChar(
                    NVS_NEXT_SLOT_KEY,
                    0
                ) %
                AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS;
        }
    }

    const String ssidKey = slotKey('s', targetSlot);
    const String passwordKey = slotKey('p', targetSlot);

    const bool saved =
        preferences.putString(
            ssidKey.c_str(),
            ssid
        ) == ssid.length() &&
        preferences.putString(
            passwordKey.c_str(),
            password
        ) == password.length();

    if (saved)
    {
        const uint8_t nextSlot =
            static_cast<uint8_t>(
                (
                    targetSlot + 1
                ) %
                AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
            );

        preferences.putUChar(
            NVS_NEXT_SLOT_KEY,
            nextSlot
        );
    }

    preferences.end();
    return saved;
}


String escapeHtml(const String &input)
{
    String escaped;
    escaped.reserve(input.length() + 16);

    for (size_t index = 0; index < input.length(); ++index)
    {
        switch (input[index])
        {
            case '&':
                escaped += F("&amp;");
                break;

            case '<':
                escaped += F("&lt;");
                break;

            case '>':
                escaped += F("&gt;");
                break;

            case '"':
                escaped += F("&quot;");
                break;

            case '\'':
                escaped += F("&#39;");
                break;

            default:
                escaped += input[index];
                break;
        }
    }

    return escaped;
}


String buildProvisioningPage()
{
    String networkOptions;
    const int networkCount = WiFi.scanNetworks(false, true);

    if (networkCount > 0)
    {
        for (int index = 0; index < networkCount; ++index)
        {
            const String ssid = escapeHtml(WiFi.SSID(index));

            if (ssid.length() == 0)
            {
                continue;
            }

            networkOptions += F("<option value=\"");
            networkOptions += ssid;
            networkOptions += F("\">");
            networkOptions += ssid;
            networkOptions += F(" (");
            networkOptions += String(WiFi.RSSI(index));
            networkOptions += F(" dBm)</option>");
        }
    }

    WiFi.scanDelete();

    String page = F(
        "<!doctype html><html lang=\"zh-CN\"><head>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,"
        "initial-scale=1\">"
        "<title>机器人 Wi-Fi 配网</title>"
        "<style>body{font-family:sans-serif;max-width:34rem;"
        "margin:2rem auto;padding:0 1rem}label,input,select,"
        "button{display:block;width:100%;box-sizing:border-box}"
        "input,select,button{font-size:1rem;padding:.75rem;"
        "margin:.4rem 0 1rem}button{cursor:pointer}</style>"
        "</head><body><h1>机器人 Wi-Fi 配网</h1>"
        "<p>选择或输入 2.4 GHz Wi-Fi。保存后设备会自动重启。"
        "最多保存 5 组网络。</p>"
        "<form method=\"post\" action=\"/save\">"
        "<label for=\"known\">扫描到的网络</label>"
        "<select id=\"known\" onchange=\"document.getElementById("
        "'ssid').value=this.value\">"
        "<option value=\"\">手动输入</option>"
    );

    page += networkOptions;
    page += F(
        "</select><label for=\"ssid\">Wi-Fi 名称</label>"
        "<input id=\"ssid\" name=\"ssid\" maxlength=\"32\" required>"
        "<label for=\"password\">Wi-Fi 密码</label>"
        "<input id=\"password\" name=\"password\" type=\"password\" "
        "maxlength=\"63\" placeholder=\"开放网络可留空\">"
        "<button type=\"submit\">保存并重启</button>"
        "</form></body></html>"
    );

    return page;
}


void redirectToProvisioningPage()
{
    const String location =
        String("http://") + WiFi.softAPIP().toString() + "/";

    provisioningServer.sendHeader(
        "Location",
        location,
        true
    );
    provisioningServer.send(
        302,
        "text/plain",
        ""
    );
}


void handleProvisioningSave()
{
    if (!provisioningServer.hasArg("ssid"))
    {
        provisioningServer.send(
            400,
            "text/plain; charset=utf-8",
            "缺少 Wi-Fi 名称"
        );
        return;
    }

    String ssid = provisioningServer.arg("ssid");
    const String password =
        provisioningServer.arg("password");
    ssid.trim();

    if (
        ssid.length() == 0 ||
        ssid.length() > MAX_SSID_LENGTH ||
        password.length() > MAX_PASSWORD_LENGTH
    )
    {
        provisioningServer.send(
            400,
            "text/plain; charset=utf-8",
            "Wi-Fi 名称或密码长度无效"
        );
        return;
    }

    if (!saveProfile(ssid, password))
    {
        provisioningServer.send(
            500,
            "text/plain; charset=utf-8",
            "保存失败，请重试"
        );
        return;
    }

    provisioningServer.send(
        200,
        "text/html; charset=utf-8",
        "<!doctype html><meta charset=\"utf-8\">"
        "<h1>保存成功</h1><p>设备即将重启并连接新网络。</p>"
    );

    Serial.printf(
        "Saved WiFi profile: %s\n",
        ssid.c_str()
    );
    restartRequested = true;
    restartRequestedAt = millis();
}


bool startProvisioningPortal()
{
    WiFi.disconnect(true, false);
    delay(100);
    WiFi.mode(WIFI_AP_STA);

    const uint32_t chipSuffix =
        static_cast<uint32_t>(ESP.getEfuseMac());
    char accessPointName[32];

    snprintf(
        accessPointName,
        sizeof(accessPointName),
        "%s%06X",
        AppConfig::WiFiProvisioning::AP_SSID_PREFIX,
        chipSuffix & 0xFFFFFF
    );

    if (
        !WiFi.softAP(
            accessPointName,
            AppConfig::WiFiProvisioning::AP_PASSWORD
        )
    )
    {
        Serial.println("Cannot start WiFi provisioning AP");
        return false;
    }

    const IPAddress accessPointIp = WiFi.softAPIP();

    dnsServer.start(
        AppConfig::WiFiProvisioning::DNS_PORT,
        "*",
        accessPointIp
    );

    provisioningServer.on(
        "/",
        HTTP_GET,
        []()
        {
            provisioningServer.send(
                200,
                "text/html; charset=utf-8",
                buildProvisioningPage()
            );
        }
    );
    provisioningServer.on(
        "/save",
        HTTP_POST,
        handleProvisioningSave
    );
    provisioningServer.on(
        "/generate_204",
        HTTP_GET,
        redirectToProvisioningPage
    );
    provisioningServer.on(
        "/hotspot-detect.html",
        HTTP_GET,
        redirectToProvisioningPage
    );
    provisioningServer.on(
        "/fwlink",
        HTTP_GET,
        redirectToProvisioningPage
    );
    provisioningServer.onNotFound(
        redirectToProvisioningPage
    );
    provisioningServer.begin();

    Serial.println();
    Serial.println("WiFi provisioning mode started");
    Serial.printf("AP SSID: %s\n", accessPointName);
    Serial.printf(
        "AP password: %s\n",
        AppConfig::WiFiProvisioning::AP_PASSWORD
    );
    Serial.print("Configuration page: http://");
    Serial.println(accessPointIp);

    while (true)
    {
        dnsServer.processNextRequest();
        provisioningServer.handleClient();

        if (
            restartRequested &&
            millis() - restartRequestedAt >= 1000
        )
        {
            Serial.println("Restarting to use saved WiFi profile");
            delay(50);
            ESP.restart();
        }

        delay(2);
    }
}


bool tryConnectProfile(
    const WiFiProfile &profile,
    unsigned long overallStart
)
{
    Serial.printf(
        "Trying saved WiFi: %s\n",
        profile.ssid.c_str()
    );

    WiFi.disconnect(false, false);
    WiFi.begin(
        profile.ssid.c_str(),
        profile.password.c_str()
    );

    const unsigned long attemptStart = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        const unsigned long now = millis();

        if (
            now - attemptStart >=
                AppConfig::WiFiProvisioning::
                    PER_NETWORK_TIMEOUT_MS ||
            now - overallStart >=
                AppConfig::WiFiProvisioning::
                    TOTAL_CONNECT_TIMEOUT_MS
        )
        {
            Serial.println("Saved WiFi connection failed");
            return false;
        }

        delay(250);
        Serial.print(".");
    }

    Serial.println();
    return true;
}


bool connectSavedProfiles()
{
    WiFiProfile profiles[
        AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
    ];
    const size_t profileCount = loadProfiles(profiles);
    const unsigned long overallStart = millis();
    bool attempted[
        AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
    ] = {};

    Serial.printf(
        "Loaded %u saved WiFi profile(s)\n",
        static_cast<unsigned int>(profileCount)
    );

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    while (
        millis() - overallStart <
        AppConfig::WiFiProvisioning::TOTAL_CONNECT_TIMEOUT_MS
    )
    {
        Serial.println("Scanning WiFi networks...");
        const int networkCount =
            WiFi.scanNetworks(false, true);
        int candidateProfiles[
            AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
        ];
        int32_t candidateRssi[
            AppConfig::WiFiProvisioning::MAX_SAVED_NETWORKS
        ];
        size_t candidateCount = 0;

        for (
            size_t profileIndex = 0;
            profileIndex < profileCount;
            ++profileIndex
        )
        {
            if (attempted[profileIndex])
            {
                continue;
            }

            int32_t strongestRssi = INT32_MIN;

            for (
                int networkIndex = 0;
                networkIndex < networkCount;
                ++networkIndex
            )
            {
                if (
                    WiFi.SSID(networkIndex) ==
                        profiles[profileIndex].ssid &&
                    WiFi.RSSI(networkIndex) > strongestRssi
                )
                {
                    strongestRssi = WiFi.RSSI(networkIndex);
                }
            }

            if (strongestRssi != INT32_MIN)
            {
                candidateProfiles[candidateCount] =
                    static_cast<int>(profileIndex);
                candidateRssi[candidateCount] = strongestRssi;
                ++candidateCount;
            }
        }

        /*
         * 先尝试信号最强的已保存网络。候选数组最多只有五项，
         * 使用简单排序可以避免引入动态分配。
         */
        for (
            size_t left = 0;
            left < candidateCount;
            ++left
        )
        {
            for (
                size_t right = left + 1;
                right < candidateCount;
                ++right
            )
            {
                if (candidateRssi[right] > candidateRssi[left])
                {
                    const int profileSwap =
                        candidateProfiles[left];
                    candidateProfiles[left] =
                        candidateProfiles[right];
                    candidateProfiles[right] = profileSwap;

                    const int32_t rssiSwap = candidateRssi[left];
                    candidateRssi[left] = candidateRssi[right];
                    candidateRssi[right] = rssiSwap;
                }
            }
        }

        WiFi.scanDelete();

        for (
            size_t candidate = 0;
            candidate < candidateCount;
            ++candidate
        )
        {
            const int profileIndex =
                candidateProfiles[candidate];
            attempted[profileIndex] = true;

            if (
                tryConnectProfile(
                    profiles[profileIndex],
                    overallStart
                )
            )
            {
                return true;
            }

            if (
                millis() - overallStart >=
                AppConfig::WiFiProvisioning::
                    TOTAL_CONNECT_TIMEOUT_MS
            )
            {
                break;
            }
        }

        if (
            millis() - overallStart <
            AppConfig::WiFiProvisioning::TOTAL_CONNECT_TIMEOUT_MS
        )
        {
            delay(
                AppConfig::WiFiProvisioning::
                    SCAN_RETRY_DELAY_MS
            );
        }
    }

    return false;
}

}  // namespace


bool wifiServiceConnect()
{
    Serial.println();
    Serial.println("Starting NVS WiFi connection");

    if (!connectSavedProfiles())
    {
        Serial.println();
        Serial.println(
            "No saved WiFi connected within 30 seconds"
        );

        return startProvisioningPortal();
    }

    Serial.println("WiFi connected");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
}


bool wifiServiceClearProfiles()
{
    Preferences preferences;

    if (!preferences.begin(NVS_NAMESPACE, false))
    {
        return false;
    }

    const bool cleared = preferences.clear();
    preferences.end();

    return cleared;
}
