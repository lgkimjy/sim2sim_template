#pragma once

// Same role as G1 States<T>: one FSM mode per derived class, driven by StateMachineCtrl.
class States {
public:
    virtual ~States() = default;

    virtual void onEnter() = 0;
    virtual void runNominal() = 0;
    virtual void checkTransition() = 0;
    virtual void runTransition() = 0;
};
