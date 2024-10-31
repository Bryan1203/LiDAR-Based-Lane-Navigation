
"use strict";

let SetExposure = require('./SetExposure.js')
let SetSleeping = require('./SetSleeping.js')
let GetCamProperties = require('./GetCamProperties.js')
let SetBrightness = require('./SetBrightness.js')
let SetROI = require('./SetROI.js')
let SetGamma = require('./SetGamma.js')
let SetBinning = require('./SetBinning.js')
let SetGain = require('./SetGain.js')
let SetBool = require('./SetBool.js')

module.exports = {
  SetExposure: SetExposure,
  SetSleeping: SetSleeping,
  GetCamProperties: GetCamProperties,
  SetBrightness: SetBrightness,
  SetROI: SetROI,
  SetGamma: SetGamma,
  SetBinning: SetBinning,
  SetGain: SetGain,
  SetBool: SetBool,
};
