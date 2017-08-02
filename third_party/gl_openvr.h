#pragma once
#define GLEW_STATIC
#define NOMINMAX
#include <iostream>
#include <string>

#include "linalg.h"   
#include "geometric.h"
#include "openvr/include/openvr.h"
#include "glew/include/GL/glew.h"
#include "glew/include/GL/wglew.h"

#ifdef _WIN64
#pragma comment(lib, "../third_party/openvr/lib/x64/openvr_api.lib") 
#pragma comment(lib, "../third_party/glew/lib/x64/glew32s.lib")
#else WIN32
#pragma comment(lib, "../third_party/openvr/lib/Win32/openvr_api.lib")
#pragma comment(lib, "../third_party/glew/lib/Win32/glew32s.lib")
#endif // WIN_32  // done DLL cases

#define check_gl {int err=glGetError(); if(err){std::cout<<__FILE__<<":"<<__LINE__<<" gl error "<<err<<std::endl;} }

inline Pose make_pose(const vr::HmdMatrix34_t & m)
{
    return{ { m.m[0][3], m.m[1][3], m.m[2][3] },
        quatfrommat({ { m.m[0][0], m.m[1][0], m.m[2][0] },{ m.m[0][1], m.m[1][1], m.m[2][1] },{ m.m[0][2], m.m[1][2], m.m[2][2] } }) };
}


class GL_OpenVR
{
public:

    struct TextureRenderBuffer
    {
        GLuint texture=0;
        GLuint depth=0;// renderbuffer
        GLuint framebuffer=0;
        int m_width=0, m_height=0;
        void clear()
        {
            glDeleteFramebuffers(1, &framebuffer);
            glDeleteTextures(1, &texture);
            glDeleteRenderbuffers(1, &depth);
            framebuffer=0;
            texture=0;
            depth=0;
        }
        void init(int width, int height)
        {
            m_width=width;
            m_height=height;
            if(!texture){glGenTextures(1, &texture); check_gl; }
            glBindTexture(GL_TEXTURE_2D, texture); check_gl;
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL); check_gl;
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); check_gl;
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); check_gl;
            if(!depth){glGenRenderbuffers(1, &depth); check_gl;}
            glBindRenderbuffer(GL_RENDERBUFFER, depth); check_gl;
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height); check_gl;
            if (!framebuffer){glGenFramebuffers(1, &framebuffer);check_gl;}
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer); check_gl;
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth); check_gl;
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);check_gl;
            int status = glCheckFramebufferStatus(GL_FRAMEBUFFER); check_gl;
            if (status != GL_FRAMEBUFFER_COMPLETE) {
                std::cout << "Framebuffer incomplete, status = " << status << std::endl;
            }
            glViewport(0, 0, m_width, m_height);check_gl;
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        };
        void bind(){
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer); check_gl;
            glViewport(0, 0, m_width, m_height); check_gl;
        }

        ~TextureRenderBuffer(){
            clear();
        }
    };
    uint2 render_dims;

    GL_OpenVR()
    {
        // Loading the SteamVR runtime
        vr::EVRInitError eError = vr::VRInitError_None;
        m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
        if (eError != vr::VRInitError_None) throw std::runtime_error("Unable to init VR runtime: " + std::string(vr::VR_GetVRInitErrorAsEnglishDescription(eError)));

        std::cout << "VR driver:  " << get_tracked_device_string(m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String) << std::endl;
        std::cout << "VR display: " << get_tracked_device_string(m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String) << std::endl;
        std::cout << "VR model:  " << get_tracked_device_string(m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String) << std::endl;

        m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
        if (!m_pRenderModels)
        {
            vr::VR_Shutdown();
            throw std::runtime_error("Unable to get render model interface: " + std::string(vr::VR_GetVRInitErrorAsEnglishDescription(eError)));
        }

        vr::RenderModel_t * model = nullptr;
        vr::RenderModel_TextureMap_t * texture = nullptr;

        // Setup Framebuffers
        m_pHMD->GetRecommendedRenderTargetSize(&render_dims.x, &render_dims.y);
        // Raise resolution for testing purposes, set to 1.4
        float res_factor = 1.0;
        render_dims.x = int(render_dims.x*res_factor);
        render_dims.y = int(render_dims.y*res_factor);

        // Setup the compositor
        m_compositor = vr::VRCompositor();
        if (!m_compositor)
        {
            throw std::runtime_error("Compositor initialization failed. See log file for details.");
            vr::VR_Shutdown();
        }
        for (int i : {0, 1}) {
            eye_buffers[i].init(render_dims.x, render_dims.y);
        }

        // turn off vsync so that the mirror window can be presented asynchronously
        wglSwapIntervalEXT(0);
    }
    ~GL_OpenVR()
    {
        std::cout << "GL_OpenVR::~GL_OpenVR" << std::endl;
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);
        glDebugMessageCallback(nullptr, nullptr);
        if (m_pHMD) vr::VR_Shutdown();
    }

    // also clears the framebuffer with current clear color
    void setup_for_eye(vr::Hmd_Eye eye, float near_clip=0.1f, float far_clip=37.0f) {
        const vr::HmdMatrix34_t head = m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
        hmd_pose = make_pose(head);
        hmd_view_matrix = get_eye_pose(eye).inverse().matrix();
        projection_matrix = get_proj_matrix(eye, near_clip, far_clip);
        glMatrixMode(GL_PROJECTION);
        check_gl;
        glLoadMatrixf((const GLfloat*)&projection_matrix);
        check_gl;
        glMatrixMode(GL_MODELVIEW);
        check_gl;
        glLoadMatrixf((const GLfloat*)&hmd_view_matrix);
        check_gl;
        eye_buffers[eye].bind();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); check_gl;
    }

	Pose get_hmd_pose() const { return hmd_pose; }
    void set_hmd_pose(Pose p) { hmd_pose = p; }

    Pose get_eye_on_hmd_pose(vr::Hmd_Eye eye){
        return make_pose(m_pHMD->GetEyeToHeadTransform(eye));
    }

    Pose get_eye_pose(vr::Hmd_Eye eye) { return get_hmd_pose() * get_eye_on_hmd_pose(eye); };
	
    float4x4 get_proj_matrix(vr::Hmd_Eye eye, float near_clip, float far_clip) 
    { return  transpose(reinterpret_cast<const float4x4 &>(m_pHMD->GetProjectionMatrix(eye, near_clip, far_clip, vr::API_OpenGL))); }

    void submit_to_hmd(vr::Hmd_Eye eye)
    {
        const vr::Texture_t texture = { (void*)(intptr_t)eye_buffers[eye].texture, vr::API_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(eye, &texture);
        //vr::VRCompositor()->PostPresentHandoff();
    }
	void wait_get_poses() { m_compositor->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, nullptr, 0); }

    void draw_mirror(){// draws mirror onto default framebuffer
        int viewport[4]{};
        int eye=1;
        glGetIntegerv(GL_VIEWPORT, viewport); check_gl;
        float w= (float)eye_buffers[eye].m_width;
        float h= (float)eye_buffers[eye].m_height;

        float sx = (viewport[2] - viewport[0]) / w;
        float sy = (viewport[3] - viewport[1]) / h;
        // scaling

        float s=std::max(sx, sy);

        float cx = (viewport[2] + viewport[0])*0.5f;
        float cy = (viewport[3] + viewport[1])*0.5f;

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); check_gl;
        glBindFramebuffer(GL_READ_FRAMEBUFFER, eye_buffers[eye].framebuffer); check_gl;
        glBlitFramebuffer(0, 0, eye_buffers[eye].m_width, eye_buffers[eye].m_height, 
            cx-w*s*0.5f, cy-h*s*0.5f, cx+w*s*0.5f, cy+h*s*0.5f,
            GL_COLOR_BUFFER_BIT, GL_LINEAR); check_gl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0); check_gl;
    }

private:
	Pose hmd_pose;
	float4x4 hmd_view_matrix;
	Pose eye_pose;
	float4x4 projection_matrix;

    vr::IVRSystem *m_pHMD;
    vr::IVRRenderModels *m_pRenderModels = nullptr;
	vr::IVRCompositor *m_compositor = nullptr; 

    TextureRenderBuffer eye_buffers[2];

    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    static inline std::string get_tracked_device_string(vr::IVRSystem * pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError * peError = NULL)
    {
        uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
        if (unRequiredBufferLen == 0)
            return "";

        char *pchBuffer = new char[unRequiredBufferLen];
        unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
        std::string sResult = pchBuffer;
        delete[] pchBuffer;
        return sResult;
    }
};
