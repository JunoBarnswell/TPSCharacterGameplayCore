import tuning from './locomotion-tuning.json' with { type: 'json' };
if (tuning.schema_version !== 1 || tuning.phase_owner !== 'server') {
  throw new RangeError('unsupported locomotion tuning schema');
}
export const locomotionTuning = Object.freeze(tuning);
export const gaitStrideLengths = Object.freeze(Object.fromEntries(
  Object.entries(tuning.gaits).map(([name, value]) => [name, value.stride_length])));
