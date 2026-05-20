#pragma once

#ifndef MATH_FIRST_ORDER_FILTER_H_
#define MATH_FIRST_ORDER_FILTER_H_

#include "math.h"
#include "Math.hpp"
#include "stm32g474xx.h"


class FirstOrderFilter
{
public:
    /**
     * @brief 初始化滤波器
     * @param update_period_s  每次调用 Update() 的时间间隔，单位：秒
     *                         例如：
     *                         10 kHz 更新 -> 0.0001f
     *                         1  kHz 更新 -> 0.001f
     * @param tau_s            时间常数，单位：秒
     *                         tau 越大，滤波越平滑，但响应越慢
     */
    void Init(float update_period_s, float tau_s)
    {
        m_Input = 0.0f;
        m_Out = 0.0f;

        m_UpdatePeriod = update_period_s;
        m_Tau = tau_s;

        m_IsInitialized = false;

        CalculateCoeff();
    }

    /**
     * @brief 设置输入值
     */
    inline void SetInput(float in)
    {
        m_Input = in;
    }

    /**
     * @brief 设置滤波输出值
     *        可用于手动指定初始状态
     */
    inline void SetResult(float out)
    {
        m_Out = out;
        m_IsInitialized = true;
    }

    /**
     * @brief 设置时间常数 tau，单位：秒
     */
    void SetTau(float tau_s)
    {
        m_Tau = tau_s;
        CalculateCoeff();
    }

    /**
     * @brief 设置滤波器更新时间间隔，单位：秒
     */
    void SetUpdatePeriod(float update_period_s)
    {
        m_UpdatePeriod = update_period_s;
        CalculateCoeff();
    }

    /**
     * @brief 获取滤波输出
     */
    inline float GetResult() const
    {
        return m_Out;
    }

    /**
     * @brief 获取时间常数
     */
    inline float GetTau() const
    {
        return m_Tau;
    }

    /**
     * @brief 获取更新时间间隔
     */
    inline float GetUpdatePeriod() const
    {
        return m_UpdatePeriod;
    }

    /**
     * @brief 获取滤波系数 a
     */
    inline float GetCoeff() const
    {
        return m_a;
    }

    /**
     * @brief 更新滤波器
     * 
     * y[k] = (1 - a) y[k-1] + a x[k]
     */
    inline void Update()
    {
        // 第一次拿到输入时，直接令输出等于输入
        if (!m_IsInitialized)
        {
            m_Out = m_Input;
            m_IsInitialized = true;
            return;
        }

        m_Out = (1.0f - m_a) * m_Out + m_a * m_Input;
    }

    /**
     * @brief 直接输入一个值并返回滤波结果
     */
    inline float Update(float in)
    {
        SetInput(in);
        Update();
        return m_Out;
    }

    /**
     * @brief 清空滤波器状态
     */
    void Clear()
    {
        m_Input = 0.0f;
        m_Out = 0.0f;
        m_IsInitialized = false;
    }

private:
    /**
     * @brief 根据 tau 和更新时间计算滤波系数
     * 
     * a = T / (tau + T)
     */
    void CalculateCoeff()
    {
        if (m_UpdatePeriod <= 0.0f)
        {
            m_a = 0.0f;
            return;
        }

        // tau <= 0 时，认为不滤波，输出直接跟随输入
        if (m_Tau <= 0.0f)
        {
            m_a = 1.0f;
            return;
        }

        m_a = m_UpdatePeriod / (m_Tau + m_UpdatePeriod);
    }

private:
    float m_Input = 0.0f;          // 当前原始输入
    float m_Out = 0.0f;            // 当前滤波输出
    float m_Tau = 0.0f;            // 时间常数，单位：秒
    float m_a = 0.0f;              // 滤波系数
    float m_UpdatePeriod = 0.0f;   // Update() 调用周期，单位：秒

    bool m_IsInitialized = false;  // 是否已经用首个输入初始化输出
};

#endif /* MATH_FIRST_ORDER_FILTER_H_ */