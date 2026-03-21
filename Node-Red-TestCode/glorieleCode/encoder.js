const now = Date.now();
const p = msg.payload;

msg.payload = {
  bucket: "testBucket2",
  precision: "ms",
  data: [{
    measurement: "encoder",
    tags: { sensor: p.sensor },
    fields: { counts: p.counts, angle_deg: p.angle_deg },
    timestamp: now
  }]
};
msg.measurement = "encoder"
return msg;
