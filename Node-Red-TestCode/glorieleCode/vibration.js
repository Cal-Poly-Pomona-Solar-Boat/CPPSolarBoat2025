const now = Date.now();
const p = msg.payload;

msg.payload = {
  bucket: "testBucket2",
  precision: "ms",
  data: [{
    measurement: "vibration",
    tags: { sensor: p.sensor },
    fields: { x_g: p.x_g, y_g: p.y_g, z_g: p.z_g },
    timestamp: now
  }]
};
msg.measurement = "vibration";
return msg;
