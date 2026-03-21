const now = Date.now();
const p = msg.payload;

msg.payload = {
  bucket: "testBucket2",
  precision: "ms",
  data: [{
    measurement: "rpm",
    tags: { sensor: p.sensor },
    fields: { rpm: p.rpm },
    timestamp: now
  }]
};
msg.measurement = "rpm"
return msg;
