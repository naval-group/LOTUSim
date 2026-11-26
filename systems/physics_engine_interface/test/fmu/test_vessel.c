/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
/*
 * A minimal FMI 2.0 Co-Simulation FMU for the tests of FmuInterface: a
 * vessel moving straight ahead at u = gain * gear * propeller, in NED,
 * keeping its attitude. Its variables follow lxdyn's FMU, as FmuInterface expects (see
 * test_fmu_interface.cpp for the modelDescription.xml), and so does
 * reset_state: only going from false to true asks for a reset.
 *
 *     1-13     outputs   Boat.x ... Boat.qk
 *     100      parameter gain           (m/s per rad/s)
 *     101      parameter gear           Enumeration: low = 1, high = 2
 *     200      input     Boat.reset_state
 *     201-213  inputs    Boat.reset_x ... Boat.reset_qk
 *     1000     input     Boat.propeller.rpm (rad/s), "propeller(rpm)" in xdyn
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fmi2Functions.h"

typedef struct {
    double state[13];
    double reset_values[13];
    int reset_input;
    int reset;
    double gain;
    int gear;
    double propeller;
    double steps;
} Vessel;

static void initial_state(Vessel* v)
{
    memset(v->state, 0, sizeof v->state);
    v->state[9] = 1.0; /* qr */
}

const char* fmi2GetTypesPlatform(void)
{
    return fmi2TypesPlatform;
}
const char* fmi2GetVersion(void)
{
    return fmi2Version;
}

fmi2Status fmi2SetDebugLogging(
    fmi2Component c,
    fmi2Boolean on,
    size_t n,
    const fmi2String cat[])
{
    (void)c;
    (void)on;
    (void)n;
    (void)cat;
    return fmi2OK;
}

fmi2Component fmi2Instantiate(
    fmi2String name,
    fmi2Type type,
    fmi2String guid,
    fmi2String resources,
    const fmi2CallbackFunctions* f,
    fmi2Boolean visible,
    fmi2Boolean logging)
{
    (void)name;
    (void)guid;
    (void)resources;
    (void)f;
    (void)visible;
    (void)logging;
    if (type != fmi2CoSimulation)
        return NULL;
    Vessel* v = calloc(1, sizeof *v);
    if (!v)
        return NULL;
    initial_state(v);
    v->gain = 0.1;
    v->gear = 1;
    return v;
}

void fmi2FreeInstance(fmi2Component c)
{
    free(c);
}

fmi2Status fmi2SetupExperiment(
    fmi2Component c,
    fmi2Boolean a,
    fmi2Real b,
    fmi2Real t0,
    fmi2Boolean d,
    fmi2Real e)
{
    (void)c;
    (void)a;
    (void)b;
    (void)t0;
    (void)d;
    (void)e;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c)
{
    (void)c;
    return fmi2OK;
}
fmi2Status fmi2ExitInitializationMode(fmi2Component c)
{
    (void)c;
    return fmi2OK;
}
fmi2Status fmi2Terminate(fmi2Component c)
{
    (void)c;
    return fmi2OK;
}

fmi2Status fmi2Reset(fmi2Component c)
{
    initial_state((Vessel*)c);
    return fmi2OK;
}

fmi2Status fmi2GetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    fmi2Real value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] >= 1 && vr[i] <= 13)
            value[i] = v->state[vr[i] - 1];
        else if (vr[i] == 100)
            value[i] = v->gain;
        else if (vr[i] == 1000)
            value[i] = v->propeller;
        else if (vr[i] == 1001)
            value[i] = v->steps;
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status fmi2SetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    const fmi2Real value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] == 100)
            v->gain = value[i];
        else if (vr[i] >= 201 && vr[i] <= 213)
            v->reset_values[vr[i] - 201] = value[i];
        else if (vr[i] == 1000)
            v->propeller = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status fmi2GetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    fmi2Integer value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] != 101)
            return fmi2Error;
        value[i] = v->gear;
    }
    return fmi2OK;
}

fmi2Status fmi2SetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    const fmi2Integer value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] != 101 || value[i] < 1 || value[i] > 2)
            return fmi2Error;
        v->gear = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2GetBoolean(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    fmi2Boolean value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] != 200)
            return fmi2Error;
        value[i] = v->reset_input;
    }
    return fmi2OK;
}

fmi2Status fmi2SetBoolean(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    const fmi2Boolean value[])
{
    Vessel* v = c;
    for (size_t i = 0; i < n; ++i) {
        if (vr[i] != 200)
            return fmi2Error;
        if (value[i] && !v->reset_input)
            v->reset = 1;
        v->reset_input = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2GetString(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    fmi2String value[])
{
    (void)c;
    (void)vr;
    (void)value;
    return n ? fmi2Error : fmi2OK;
}

fmi2Status fmi2SetString(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t n,
    const fmi2String value[])
{
    (void)c;
    (void)vr;
    (void)value;
    return n ? fmi2Error : fmi2OK;
}

fmi2Status
fmi2DoStep(fmi2Component c, fmi2Real t, fmi2Real h, fmi2Boolean no_prior)
{
    (void)t;
    (void)no_prior;
    Vessel* v = c;
    if (v->reset) {
        memcpy(v->state, v->reset_values, sizeof v->state);
        v->reset = 0;
    }
    double* s = v->state;
    /* Heading from the quaternion, the vessel being level. */
    const double psi = 2.0 * atan2(s[12], s[9]);
    const double u = v->gain * v->gear * v->propeller;
    s[0] += u * cos(psi) * h;
    s[1] += u * sin(psi) * h;
    s[3] = u;
    s[4] = s[5] = s[6] = s[7] = s[8] = 0.0;
    v->steps += 1.0;
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c)
{
    (void)c;
    return fmi2Error;
}
