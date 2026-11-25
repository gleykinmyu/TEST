#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include "joystick_axis.h"
#include "joystick_ema.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_eth_netif_glue.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace {
constexpr adc_channel_t channels[]={ADC_CHANNEL_3,ADC_CHANNEL_0,ADC_CHANNEL_7}; // GPIO39,36,35
constexpr gpio_num_t button=GPIO_NUM_14;
constexpr int average_count=16;
constexpr size_t settings_v1=10; // zero[3], dead, curve, padding
struct Settings {
    uint16_t zero[3]={2048,2048,2048};
    uint16_t dead=80;
    uint8_t curve=0;
    uint8_t pad=0;
    uint16_t alpha=250; // thousandths, 0.25
};
static_assert(offsetof(Settings,alpha)==settings_v1,"NVS v1 prefix");
static_assert(sizeof(Settings)==12,"NVS v2 size");
Settings settings;
joystick::Axis axes[3];
joystick::Ema filters[3];
int raw[3]{}, filtered[3]{}, values[3]{};
bool pressed=false;
SemaphoreHandle_t lock;
adc_oneshot_unit_handle_t adc;
httpd_handle_t server;
bool valid_base(const Settings& s){
    if(s.curve>2||s.dead>400)return false;
    for(auto z:s.zero)if(z<201||z>3894)return false;
    return true;
}
bool valid(const Settings& s){return valid_base(s)&&s.alpha>=10&&s.alpha<=1000;}
bool parse_alpha(const char* s,uint16_t& milli){
    if(!s||*s<'0'||*s>'9')return false;
    unsigned whole=0;const char* p=s;
    while(*p>='0'&&*p<='9'){whole=whole*10u+unsigned(*p-'0');if(whole>1)return false;++p;}
    unsigned frac=0,place=100;
    if(*p=='.'){++p;if(*p<'0'||*p>'9')return false;while(*p>='0'&&*p<='9'){if(place){frac+=unsigned(*p-'0')*place;place/=10;}++p;}}
    else if(*p)return false;
    unsigned v=whole*1000u+frac;
    if(v<10||v>1000)return false;
    milli=uint16_t(v);return true;
}
bool read_mean(adc_channel_t ch,int& out){
    int sum=0;
    for(int i=0;i<average_count;i++){int v;if(adc_oneshot_read(adc,ch,&v)!=ESP_OK)return false;sum+=v;}
    out=(sum+average_count/2)/average_count;
    return true;
}
void apply(){
    for(int i=0;i<3;i++){
        axes[i].calibrate(settings.zero[i]);axes[i].deadzone(settings.dead);axes[i].exponent(static_cast<joystick::Curve>(settings.curve));
        filters[i].alpha(settings.alpha);
        if(filters[i].primed())values[i]=axes[i].compute(filters[i].value());
    }
}
bool save(const Settings& s){
    nvs_handle_t h;
    if(nvs_open("joystick",NVS_READWRITE,&h)!=ESP_OK)return false;
    auto e=nvs_set_blob(h,"settings",&s,sizeof(s));
    if(e==ESP_OK)e=nvs_commit(h);
    nvs_close(h);return e==ESP_OK;
}
void load(){
    nvs_handle_t h;
    if(nvs_open("joystick",NVS_READONLY,&h)!=ESP_OK)return;
    Settings s;size_t n=sizeof(s);auto e=nvs_get_blob(h,"settings",&s,&n);
    if(e==ESP_OK&&n==settings_v1&&valid_base(s)){s.pad=0;s.alpha=250;settings=s;}
    else if(e==ESP_OK&&n==sizeof(s)&&valid(s))settings=s;
    nvs_close(h);
}
void sample_task(void*){
    bool candidate=false;unsigned stable=0;
    TickType_t last=xTaskGetTickCount();
    for(;;){
        xSemaphoreTake(lock,portMAX_DELAY);
        for(int i=0;i<3;i++){
            int mean;if(!read_mean(channels[i],mean))continue;
            raw[i]=mean;filtered[i]=filters[i].update(uint16_t(mean));values[i]=axes[i].compute(uint16_t(filtered[i]));
        }
        bool b=!gpio_get_level(button);
        if(b!=candidate){candidate=b;stable=0;}else if(stable<2)++stable;
        if(stable>=2)pressed=candidate;
        xSemaphoreGive(lock);
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
    }
}
esp_err_t state(httpd_req_t* r){
    char buf[384];xSemaphoreTake(lock,portMAX_DELAY);
    snprintf(buf,sizeof(buf),"{\"raw\":[%d,%d,%d],\"filtered\":[%d,%d,%d],\"value\":[%d,%d,%d],\"center\":[%u,%u,%u],\"button\":%s,\"curve\":%u,\"dead\":%u,\"alpha\":%u.%03u}",raw[0],raw[1],raw[2],filtered[0],filtered[1],filtered[2],values[0],values[1],values[2],settings.zero[0],settings.zero[1],settings.zero[2],pressed?"true":"false",settings.curve,settings.dead,settings.alpha/1000,settings.alpha%1000);
    xSemaphoreGive(lock);httpd_resp_set_type(r,"application/json");httpd_resp_set_hdr(r,"Cache-Control","no-store");return httpd_resp_sendstr(r,buf);
}
esp_err_t change(httpd_req_t* r){
    char query[96],c[8],d[8],a[16];unsigned cv,dv;uint16_t av;char extra;
    if(httpd_req_get_url_query_str(r,query,sizeof(query))!=ESP_OK||httpd_query_key_value(query,"curve",c,sizeof(c))!=ESP_OK||httpd_query_key_value(query,"dead",d,sizeof(d))!=ESP_OK||httpd_query_key_value(query,"alpha",a,sizeof(a))!=ESP_OK||sscanf(c,"%u%c",&cv,&extra)!=1||sscanf(d,"%u%c",&dv,&extra)!=1||!parse_alpha(a,av)||cv>2||dv>400)return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid settings");
    xSemaphoreTake(lock,portMAX_DELAY);Settings s=settings;s.curve=cv;s.dead=dv;s.alpha=av;bool ok=save(s);if(ok){settings=s;apply();}xSemaphoreGive(lock);
    return ok?state(r):httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS write failed");
}
esp_err_t zero(httpd_req_t* r){
    xSemaphoreTake(lock,portMAX_DELAY);Settings s=settings;int mean[3]{};bool ok=true;
    for(int i=0;i<3&&ok;i++)ok=read_mean(channels[i],mean[i]);
    if(ok)for(int i=0;i<3;i++)s.zero[i]=uint16_t(mean[i]);
    if(!ok||!valid(s)){xSemaphoreGive(lock);return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid center");}
    ok=save(s);
    if(ok){settings=s;for(int i=0;i<3;i++){filters[i].reset(uint16_t(mean[i]));raw[i]=filtered[i]=mean[i];}apply();}
    xSemaphoreGive(lock);
    return ok?state(r):httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS write failed");
}
esp_err_t ota(httpd_req_t* r){
    const auto* p=esp_ota_get_next_update_partition(nullptr);
    if(!p||r->content_len==0||r->content_len>p->size)return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid image size");
    esp_ota_handle_t h;
    if(esp_ota_begin(p,r->content_len,&h)!=ESP_OK)return httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"OTA begin failed");
    char buf[2048];size_t left=r->content_len;
    while(left){int n=httpd_req_recv(r,buf,std::min(left,sizeof(buf)));if(n<=0||esp_ota_write(h,buf,n)!=ESP_OK){esp_ota_abort(h);return httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"OTA transfer failed");}left-=n;}
    if(esp_ota_end(h)!=ESP_OK||esp_ota_set_boot_partition(p)!=ESP_OK)return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Image validation failed");
    httpd_resp_sendstr(r,"OK, rebooting");
    vTaskDelay(pdMS_TO_TICKS(500));esp_restart();return ESP_OK;
}
const char page[]=R"HTML(<!doctype html><html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Joystick</title>
<style>body{font:16px system-ui;max-width:620px;margin:24px auto;padding:0 16px;background:#17202b;color:#eee}meter{width:100%;height:28px}canvas{width:100%;background:#263342}button,input,select{font:inherit;margin:8px 0}</style>
<h1>Joystick</h1><p id="status">Connecting</p><div id="axes"></div><p>Button: <b id="btn">—</b></p>
<select id="curve"><option value="0">Linear</option><option value="1">Square</option><option value="2">Cubic</option></select>
<label>Dead zone <input id="dead" type="number" min="0" max="400"></label>
<label>EMA <input id="alpha" type="number" min="0.01" max="1" step="0.01"></label><button id="save">Save</button><canvas id="graph" width="500" height="200"></canvas>
<button id="zero">Set current position as zero</button><hr><h2>OTA</h2><input id="file" type="file" accept=".bin"><button id="upload">Upload firmware</button><p id="ota"></p>
<script>
const $=x=>document.getElementById(x);$('axes').innerHTML=['X','Y','Z'].map((a,i)=>'<p>'+a+' <b id="v'+i+'"></b> <small id="r'+i+'"></small><meter id="m'+i+'" min="-32767" max="32767" value="0"></meter></p>').join('');
function graph(){let c=$('graph'),g=c.getContext('2d'),w=c.width,h=c.height;g.clearRect(0,0,w,h);g.strokeStyle='#8795a5';g.beginPath();g.moveTo(0,h/2);g.lineTo(w,h/2);g.moveTo(w/2,0);g.lineTo(w/2,h);g.stroke();g.strokeStyle='#69baff';g.beginPath();for(let i=0;i<=w;i++){let x=i/w*2-1,y=Math.sign(x)*Math.pow(Math.abs(x),1+Number($('curve').value));g.lineTo(i,(1-y)*h/2)}g.stroke()}
let init=false,busy=false;async function poll(){try{if(!busy){let r=await fetch('/state',{cache:'no-store'});if(!r.ok)throw Error();let s=await r.json();$('status').textContent='Connected';if(!init){$('curve').value=s.curve;$('dead').value=s.dead;$('alpha').value=s.alpha;init=true;graph()}s.value.forEach((v,i)=>{$('v'+i).textContent=v;$('m'+i).value=v;$('r'+i).textContent='ADC '+s.raw[i]+' / filt '+s.filtered[i]+' / zero '+s.center[i]});$('btn').textContent=s.button?'Pressed':'Released'}}catch(e){$('status').textContent='Disconnected'}finally{setTimeout(poll,50)}}
async function post(url){try{let r=await fetch(url,{method:'POST'});if(!r.ok)throw Error(await r.text());init=false}catch(e){alert(e.message)}}
$('curve').onchange=graph;$('save').onclick=()=>post('/settings?curve='+$('curve').value+'&dead='+$('dead').value+'&alpha='+$('alpha').value);$('zero').onclick=()=>post('/zero');
$('upload').onclick=async()=>{let f=$('file').files[0];if(!f||busy||!confirm('Update firmware?'))return;busy=true;$('ota').textContent='Uploading...';try{let r=await fetch('/ota',{method:'POST',body:f});$('ota').textContent=await r.text()}catch(e){$('ota').textContent='Connection lost; check device after reboot'}finally{busy=false;init=false}};poll();graph();
</script></html>)HTML";
esp_err_t index(httpd_req_t* r){httpd_resp_set_type(r,"text/html; charset=utf-8");return httpd_resp_sendstr(r,page);}
void http_start(){httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();cfg.stack_size=8192;ESP_ERROR_CHECK(httpd_start(&server,&cfg));
    const httpd_uri_t routes[]={{"/",HTTP_GET,index,nullptr},{"/state",HTTP_GET,state,nullptr},{"/settings",HTTP_POST,change,nullptr},{"/zero",HTTP_POST,zero,nullptr},{"/ota",HTTP_POST,ota,nullptr}};
    for(auto& route:routes)ESP_ERROR_CHECK(httpd_register_uri_handler(server,&route));
}
void network(){
    ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_16,GPIO_MODE_OUTPUT));gpio_set_level(GPIO_NUM_16,0);vTaskDelay(pdMS_TO_TICKS(100));gpio_set_level(GPIO_NUM_16,1);vTaskDelay(pdMS_TO_TICKS(200));
    esp_netif_config_t nc=ESP_NETIF_DEFAULT_ETH();auto* netif=esp_netif_new(&nc);
    eth_mac_config_t mc=ETH_MAC_DEFAULT_CONFIG();eth_phy_config_t pc=ETH_PHY_DEFAULT_CONFIG();pc.phy_addr=1;pc.reset_gpio_num=-1;
    eth_esp32_emac_config_t ec=ETH_ESP32_EMAC_DEFAULT_CONFIG();ec.smi_gpio.mdc_num=23;ec.smi_gpio.mdio_num=18;ec.clock_config.rmii.clock_mode=EMAC_CLK_EXT_IN;ec.clock_config.rmii.clock_gpio=EMAC_CLK_IN_GPIO;
    auto* mac=esp_eth_mac_new_esp32(&ec,&mc);auto* phy=esp_eth_phy_new_lan87xx(&pc);assert(mac&&phy&&netif);
    esp_eth_config_t cfg=ETH_DEFAULT_CONFIG(mac,phy);esp_eth_handle_t eth=nullptr;ESP_ERROR_CHECK(esp_eth_driver_install(&cfg,&eth));uint8_t addr[6];ESP_ERROR_CHECK(esp_read_mac(addr,ESP_MAC_ETH));ESP_ERROR_CHECK(esp_eth_ioctl(eth,ETH_CMD_S_MAC_ADDR,addr));ESP_ERROR_CHECK(esp_netif_attach(netif,esp_eth_new_netif_glue(eth)));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_ETH_GOT_IP,[](void*,esp_event_base_t,int32_t,void* data){auto* e=static_cast<ip_event_got_ip_t*>(data);ESP_LOGI("joystick","IP: " IPSTR,IP2STR(&e->ip_info.ip));},nullptr));
    ESP_ERROR_CHECK(esp_eth_start(eth));
}
}
extern "C" void app_main(){
    ESP_ERROR_CHECK(nvs_flash_init());load();lock=xSemaphoreCreateMutex();assert(lock);
    adc_oneshot_unit_init_cfg_t uc{};uc.unit_id=ADC_UNIT_1;ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc,&adc));
    adc_oneshot_chan_cfg_t cc{};cc.atten=ADC_ATTEN_DB_12;cc.bitwidth=ADC_BITWIDTH_12;for(auto ch:channels)ESP_ERROR_CHECK(adc_oneshot_config_channel(adc,ch,&cc));
    gpio_config_t bc{};bc.pin_bit_mask=1ULL<<button;bc.mode=GPIO_MODE_INPUT;bc.pull_up_en=GPIO_PULLUP_ENABLE;ESP_ERROR_CHECK(gpio_config(&bc));
    apply();assert(xTaskCreate(sample_task,"sample",4096,nullptr,5,nullptr)==pdPASS);network();http_start();
}
