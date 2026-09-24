export const ACTION_CHANNEL_NAMES = Object.freeze([
  "locomotion",
  "upper_body_action",
  "additive_reaction",
  "full_body_override",
  "life_override",
]);

const ACTION_CHANNELS = ACTION_CHANNEL_NAMES.filter((name) => name !== "locomotion");

function cloneSnapshot(snapshot) {
  return Object.fromEntries(ACTION_CHANNEL_NAMES.map((name) => [
    name,
    snapshot[name] ? { ...snapshot[name] } : {
      state: "none",
      active: false,
      start_tick: 0,
      end_tick: null,
      sequence: 0,
      event_id: "",
      blend_semantic: "none",
    },
  ]));
}

export class ActionChannelSnapshotCursor {
  constructor() {
    this.channels = new Map();
  }

  reset() {
    this.channels.clear();
  }

  apply(snapshot) {
    if (!snapshot || typeof snapshot !== "object") {
      throw new TypeError("action channel snapshot must be an object");
    }
    const startedEvents = [];
    for (const name of ACTION_CHANNEL_NAMES) {
      const incoming = snapshot[name];
      if (!incoming || !Number.isInteger(incoming.sequence) ||
          typeof incoming.event_id !== "string" || typeof incoming.active !== "boolean") {
        throw new TypeError(`action channel '${name}' has invalid sequence or state`);
      }
      const current = this.channels.get(name);
      if (current && incoming.sequence < current.sequence) continue;
      if (current && incoming.sequence === current.sequence) {
        const sameRevision = incoming.event_id === current.event_id &&
          incoming.state === current.state &&
          incoming.active === current.active &&
          incoming.start_tick === current.start_tick &&
          incoming.end_tick === current.end_tick &&
          incoming.blend_semantic === current.blend_semantic;
        if (!sameRevision) {
          throw new RangeError(`action channel '${name}' reused sequence ${incoming.sequence}`);
        }
        continue;
      }
      const next = { ...incoming };
      this.channels.set(name, next);
      if (ACTION_CHANNELS.includes(name) && next.active && next.state !== "none") {
        startedEvents.push({ channel: name, ...next });
      }
    }
    return {
      channels: cloneSnapshot(Object.fromEntries(this.channels)),
      startedEvents,
    };
  }
}
