//
//
// The code is currently set up to link with the compiled versions of libovr for visual studio 2015.
// Has not been tested on other compilers and systems at this time.
// Has been tested only with the june2017 version of ovr sdk.  
// Suggest only using release, but if using Debug it may also be necessary to compile LibOVR 
// if pre-compiled binaries were not included with the SDK download.   
// Furthermore, it will probably be necessary to rebuild LibOVR anyways with the right configuration.
//
// It may be necessary to open up the visual studio solution for the OVR SDK and modify 
// some settings to be compatable with the projects in this repo and librealsense.
// in particular if you open up another instance of visual studio with:
//   ...\hand_tracking_samples\third_party\OculusSDK\Samples\Projects\Windows\VS2015\Samples.sln
// and modify the project settings for project LibOVR
//   ...\hand_tracking_samples\third_party\OculusSDK\LibOVR\Projects\Windows\VS2015\LibOVR.vcxproj
// assuming you are building for Configuration Release and Platform Win32,
// look for properties => c/c++ => Code Generation => Runtime Library 
// and change it from Multi-threaded (/MT) to Multi-threaded DLL (/MD)
//
// Alternatively, it may be possible to modify the settings of the other projects to use the same settings as LibOVR.
//

#pragma once

#include "../third_party/geometric.h"
#include "dxwin.h"

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib,"../third_party/OculusSDK/LibOVR/Lib/Windows/x64/Debug/VS2015/libovr.lib")
#else
#pragma comment(lib,"../third_party/OculusSDK/LibOVR/Lib/Windows/x64/Release/VS2015/libovr.lib")
#endif

#else // win32
#ifdef _DEBUG
#pragma comment(lib,"../third_party/OculusSDK/LibOVR/Lib/Windows/Win32/Debug/VS2015/libovr.lib")
#else
#pragma comment(lib,"../third_party/OculusSDK/LibOVR/Lib/Windows/Win32/Release/VS2015/libovr.lib")
#endif
#endif __WIN64 vs win32
// also be sure to set c++ code generation settings runtime library to be multi-threaded (not multi-threaded dll))

// Include the Oculus SDK
#include "../third_party/OculusSDK/LibOVR/Include/OVR_CAPI_D3D.h"

float4      fromovr(const ovrQuatf    &v) { return (float4&)v;}
float3      fromovr(const ovrVector3f &v) { return (float3&)v; }
Pose        fromovr(const ovrPosef    &p) { return{ fromovr(p.Position),fromovr(p.Orientation) }; }  // different binary layouts  
float4x4    fromovr(const ovrMatrix4f &m) { return  (float4x4&)m; }
ovrQuatf    toovr(const float4      &v) { return{ v.x,v.y,v.z,v.w }; }    // implementation here is more explicit due to memory alignment of ovr types
ovrVector3f toovr(const float3      &v) { return{ v.x,v.y,v.z }; }
ovrPosef    toovr(const Pose        &p) { return  ovrPosef{ toovr(p.orientation), toovr(p.position) }; }
ovrMatrix4f toovr(const float4x4    &m) { return{ m[0][0],m[0][1],m[0][2],m[0][3],m[1][0],m[1][1],m[1][2],m[1][3],m[2][0],m[2][1],m[2][2],m[2][3],m[3][0],m[3][1],m[3][2],m[3][3] }; }


template<typename T> void Release(T *&obj) // that COM style 
{
	if (obj) 
		obj->Release();
	obj = nullptr;
}

class OVRWin : public DXWin
{
public:
	//------------------------------------------------------------
	// the following snippet adapted from the oculus sdk tinyroom example:
	// ovrSwapTextureSet wrapper class that also maintains the render target views
	// needed for D3D11 rendering.
	struct OculusTexture  
	{
		//OculusTexture(OculusTexture&) = delete;

		ovrSession               Session ;
		ovrTextureSwapChain      TextureChain;
		std::vector<ID3D11RenderTargetView*> TexRtv;

		~OculusTexture()
		{
			return;  // till we make sure we aren't ever copying this struct
			for (int i = 0; i < (int)TexRtv.size(); ++i)
			{
				Release(TexRtv[i]);
			}
			if (TextureChain)
			{
				ovr_DestroyTextureSwapChain(Session, TextureChain);
			}
		}

		ID3D11RenderTargetView* GetRTV()
		{
			int index = 0;
			ovr_GetTextureSwapChainCurrentIndex(Session, TextureChain, &index);
			return TexRtv[index];
		}

		// Commit changes
		void Commit()
		{
			ovr_CommitTextureSwapChain(Session, TextureChain);
		}
	};
	OculusTexture *MakeOculusTexture(ovrSession session, int sizeW, int sizeH)
	{
		ovrTextureSwapChain      TextureChain;
		std::vector<ID3D11RenderTargetView*> TexRtv;

		ovrTextureSwapChainDesc desc = {};
		desc.Type = ovrTexture_2D;
		desc.ArraySize = 1;
		desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
		desc.Width = sizeW;
		desc.Height = sizeH;
		desc.MipLevels = 1;
		desc.SampleCount = 1;
		desc.MiscFlags = ovrTextureMisc_DX_Typeless;
		desc.BindFlags = ovrTextureBind_DX_RenderTarget;
		desc.StaticImage = ovrFalse;

		ovrResult result = ovr_CreateTextureSwapChainDX(session, dxdevice, &desc, &TextureChain);
		if (!OVR_SUCCESS(result))
			throw "fail to create ovr swap chain";

		int textureCount = 0;
		ovr_GetTextureSwapChainLength(session, TextureChain, &textureCount);
		for (int i = 0; i < textureCount; ++i)
		{
			ID3D11Texture2D* tex = nullptr;
			ovr_GetTextureSwapChainBufferDX(session, TextureChain, i, IID_PPV_ARGS(&tex));
			D3D11_RENDER_TARGET_VIEW_DESC rtvd = {};
			rtvd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			rtvd.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
			ID3D11RenderTargetView* rtv;
			dxdevice->CreateRenderTargetView(tex, &rtvd, &rtv);
			TexRtv.push_back(rtv);
			tex->Release();
		}

		return new OculusTexture({ session,TextureChain,TexRtv });
	}

	// Initialize these to nullptr here to handle device lost failures cleanly
	ovrMirrorTexture mirrorTexture = nullptr;
	OculusTexture  * pEyeRenderTexture[2] = { nullptr, nullptr };
	ID3D11DepthStencilView* pEyeDepthBuffer[2] = { nullptr, nullptr };
	ovrMirrorTextureDesc mirrorDesc = {};
	long long frameIndex = 0;

	ovrSession session;
	ovrGraphicsLuid luid;
	ovrRecti         eyeRenderViewport[2];
	ovrHmdDesc hmdDesc;

	OVRWin(std::string title) :DXWin(title.c_str(), { 960,540 })   , frameIndex(0)
	{
		// Initializes LibOVR, and the Rift
		ovrInitParams initParams = { ovrInit_RequestVersion, OVR_MINOR_VERSION, NULL, 0, 0 };
		ovrResult result = ovr_Initialize(&initParams);
		if(!OVR_SUCCESS(result))
			throw "Failed to initialize libOVR.";

		result = ovr_Create(&session, &luid);
		if (!OVR_SUCCESS(result))
			throw "fialed over create"; 

		hmdDesc = ovr_GetHmdDesc(session);  // ovrHmdDesc

		for (int eye = 0; eye < 2; ++eye)
		{
			ovrSizei idealSize = ovr_GetFovTextureSize(session, (ovrEyeType)eye, hmdDesc.DefaultEyeFov[eye], 1.0f);
			pEyeRenderTexture[eye] = new OculusTexture();
			pEyeRenderTexture[eye] = MakeOculusTexture(session, idealSize.w, idealSize.h);
			pEyeDepthBuffer[eye] = CreateDepthStencilView( { idealSize.w, idealSize.h });
			eyeRenderViewport[eye].Pos.x = 0;
			eyeRenderViewport[eye].Pos.y = 0;
			eyeRenderViewport[eye].Size = idealSize;
			if (!pEyeRenderTexture[eye]->TextureChain)
			{
				throw "Failed to create texture.";
			}
		}

		// Create a mirror to see on the monitor.
		mirrorDesc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
		mirrorDesc.Width = res.x;
		mirrorDesc.Height = res.y;
		result = ovr_CreateMirrorTextureDX(session, dxdevice, &mirrorDesc, &mirrorTexture);
		if (!OVR_SUCCESS(result))
		{
			throw "Failed to create mirror texture.";
		}
	}
	ovrSessionStatus sessionStatus;
	float4 clearcolor = { 0, 0.25f, 0.25f, 0.0f };

	int WindowUp()
	{
		auto rv = DXWin::WindowUp();
		ovr_GetSessionStatus(session, &sessionStatus);
		if (sessionStatus.ShouldQuit)
			return false;
		if (sessionStatus.ShouldRecenter)
			ovr_RecenterTrackingOrigin(session);
		return rv;
	}

	Pose last_hmd_pose;  // just the last eyepose for now
	void RenderVR(Pose game_camera, const std::vector<Mesh*> &meshes)
	{
		ovrResult result;
		// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
		ovrEyeRenderDesc eyeRenderDesc[2];
		eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
		eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);

		// Get both eye poses simultaneously, with IPD offset already included. 
		ovrPosef         EyeRenderPose[2];
		ovrVector3f      HmdToEyeOffset[2] = { eyeRenderDesc[0].HmdToEyeOffset, eyeRenderDesc[1].HmdToEyeOffset };

		double sensorSampleTime;    // sensorSampleTime is fed into the layer later
		ovr_GetEyePoses(session, frameIndex, ovrTrue, HmdToEyeOffset, EyeRenderPose, &sensorSampleTime);



		// Render Scene to Eye Buffers
		for (int eye = 0; eye < 2; ++eye)
		{
			// Clear and set up rendertarget
			RenderTarget rt = {int2(eyeRenderViewport[eye].Size.w,eyeRenderViewport[eye].Size.h),nullptr,pEyeRenderTexture[eye]->GetRTV(), pEyeDepthBuffer[eye] };
			SetUpAndClear(rt,clearcolor );


			DXWin::ConstantBuffer cb;     // this is the collection of constants we make available to our dx vertex and pixel shaders 
			cb.projection = (float4x4&)(ovrMatrix4f_Projection(eyeRenderDesc[eye].Fov, 0.02f, 100.0f, ovrProjection_None));
			auto camera = last_hmd_pose = game_camera * fromovr(EyeRenderPose[eye]);
			cb.camerap = camera.position;
			cb.cameraq = camera.orientation;
			DrawMeshes(cb, meshes);

			// Commit rendering to the swap chain
			pEyeRenderTexture[eye]->Commit();
		}

		// Initialize our single full screen Fov layer.
		ovrLayerEyeFov ld = {};
		ld.Header.Type = ovrLayerType_EyeFov;
		ld.Header.Flags = 0;

		for (int eye = 0; eye < 2; ++eye)
		{
			ld.ColorTexture[eye] = pEyeRenderTexture[eye]->TextureChain;
			ld.Viewport[eye] = eyeRenderViewport[eye];
			ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
			ld.RenderPose[eye] = EyeRenderPose[eye];
			ld.SensorSampleTime = sensorSampleTime;
		}

		ovrLayerHeader* layers = &ld.Header;
		result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);
		// exit the rendering loop if submit returns an error, will retry on ovrError_DisplayLost
		if (!OVR_SUCCESS(result))
			throw "failed to submit frame";

		frameIndex++;
		// Render mirror
		ID3D11Texture2D* tex = nullptr;
		ovr_GetMirrorTextureBufferDX(session, mirrorTexture, IID_PPV_ARGS(&tex));
		ID3D11Texture2D* pBackBuffer = nullptr;
		dxswapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer)) && VERIFY;  //("dxswapchain->GetBuffer");
		dxcontext->CopyResource(pBackBuffer, tex);
		pBackBuffer->Release();
		tex->Release();
		dxswapchain->Present(0, 0);
	}
};
