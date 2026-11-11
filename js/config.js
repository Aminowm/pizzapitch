// Настройки сайта. Должны совпадать с base_lora.ino (MQTT_HOST и TOPIC_ROOT).
window.BARLAUSHY_CONFIG = {
  // публичный брокер EMQX: база шлёт на порт 1883, сайт (https) слушает через защищённый WebSocket 8084
  mqttUrl: "wss://broker.emqx.io:8084/mqtt",
  topicRoot: "barlaushy/zhz1407",
  // через сколько секунд без свежих данных предложить демо
  demoHintAfterSec: 8,
};
