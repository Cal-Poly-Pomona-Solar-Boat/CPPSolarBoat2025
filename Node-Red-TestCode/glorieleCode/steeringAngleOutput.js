const now = Date.now();
const p = msg.payload;

msg.payload = {
  bucket: "testBucket2",
  precision: "ms",
  data: [{
    measurement: "steering_angle",
    tags: { sensor: p.sensor },
    fields: { degrees: p.degrees },
    timestamp: now
  }]
};

msg.measurement = "steering_angle";
return msg;
