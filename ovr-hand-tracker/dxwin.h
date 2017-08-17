// 
//   dxwin.h
//
// trying for smallest possible d3d11 on windows framework for writing quick graphics test apps but need d3d11.
// All in one header file.  No extra downloads, installs, cmake, .DLLs, .LIBs. etc...
//
//

#ifndef  SANDBOX_DXWIN_H
#define  SANDBOX_DXWIN_H


#include <assert.h>
#include <functional>
#include <vector>
#include <string>
#include <map>

#include "../third_party/mswin.h"
#include "../third_party/mesh.h"
#include <d3d11_1.h>
#include <d3dcompiler.h>

#include <cstring>
#include <cstdarg>   // For va_list, va_start, ...
#include <cstdio>    // For vsnprintf

#ifdef WIN32
#pragma comment(lib,"d3d11.lib")
#pragma comment(lib,"d3dcompiler.lib")
#pragma comment(lib,"dxguid.lib")
#endif

template<class K, class V> V map_lookup_default(const std::map<K, V> &m, const K& k, V d) { return m.find(k) != m.end() ? m[k] : d; }

struct Material
{
	std::string diffusemap;
	std::string normalmap;
	std::string pixelshader;
};
template<class F> void visit_fields(Material &o, F f) { f("diffusemap", o.diffusemap); f("normalmap", o.normalmap); f("pixelshader", o.pixelshader); }

// not sure if best way to do this:
#include "dxshaders.h"  // defines shaders in a big string (char*) dxshaders



inline float4x4 MatrixPerspectiveFovAspect(float fovy, float aspect, float zn, float zf) { auto h = 1 / tan(fovy / 2), zr = zf / (zn - zf); return{ { h / aspect, 0, 0, 0 }, { 0, h, 0, 0 }, { 0, 0, zr, -1 }, { 0, 0, zn*zr, 0 } }; }
#define OFFSET(Class,Member)  (((char*) (&(((Class*)NULL)-> Member )))- ((char*)NULL))

//from mesh.h:  struct Vertex{float3 position; float4 orientation; float2 texcoord;};

class DXWin : public MSWin
{
public:

	int                     vsync = 1;


	ID3D11Device*           dxdevice           = nullptr;
	ID3D11Device1*          dxdevice1          = nullptr;
	ID3D11DeviceContext*    dxcontext          = nullptr;
	IDXGISwapChain*         dxswapchain        = nullptr;


	struct ConstantBuffer  // struct matches global var struct used by shaders
	{
		float4 hack = { 1, 1, 0, 1 };
		float4x4 projection;
		float3 camerap; float unusedc;  // seems to need like float3  so either float4 or have extra 32bit float with float3
		float4 cameraq = { 0, 0, 0, 1 };
		float3 meshp; float unusedm;
		float4 meshq = { 0, 0, 0, 1 };
		float4 lightposn = { -2, -4, 3, 1 };
		ConstantBuffer(){}
		ConstantBuffer(float fov, float aspect) :projection(transpose(MatrixPerspectiveFovAspect(fov*3.14f / 180.0f, aspect, 0.05f, 50.0f))){}
	};

	ID3D11ShaderResourceView *MakeTexCheckerboard()
	{
		int w = 64, h = 64;
		std::vector<int> image(64 * 64, 0);
		for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
			image[i*w + j] = ((i & 8) == (j & 8)) ? 0x00000000 : -1;
		return MakeTex((unsigned char*)image.data(), w, h);
	}
	ID3D11ShaderResourceView *MakeTexWav()
	{
		int w = 128, h = 128;
		std::vector<float> hmap(w*h, 0);
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			float fx = (float)x / float(w) -0.5f ;
			float fy = (float)y / float(h) - 0.5f;
			hmap[y*w + x] = 0.5f + std::max(0.0f,cosf( sqrtf(fx*fx+fy*fy)*1.1f*3.14f  )) / 2.0f / 1.0f;  // 1/10 max slopes
//			hmap[y*w + x] = 0.5f + (cosf(((float)x / (float)(w))*3.14f*2.0f * 3) + cosf( ((float)y / (float)h)*3.14f*2.0f * 3)) / 4.0f / 10.0f;  // 1/10 max slopes
		}
		std::vector<unsigned char> image(w * h * 4, 0);
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			int xp = (x + w - 1) % w, xn = (x + 1) % w, yp = (y + h - 1) % h, yn = (y + 1) % h;
			auto n = normalize(float3( (hmap[y*w + xp] - hmap[y*w + xn]), (hmap[yp*w + x] - hmap[yn*w + x]), 2.0f/w ))*127.0f + float3(128,128,128);
			image[(y*w + x) * 4 + 0] = (unsigned char)n.x;
			image[(y*w + x) * 4 + 1] = (unsigned char)n.y;
			image[(y*w + x) * 4 + 2] = (unsigned char)n.z;
			image[(y*w + x) * 4 + 3] = (unsigned char) (hmap[y*w+x]*255.0f );
		}
		return MakeTex((unsigned char*)image.data(), w, h);
	}
	int2 TexDims(ID3D11ShaderResourceView *srv)
	{
		D3D11_TEXTURE2D_DESC tdesc;
		ID3D11Texture2D *tex;
		srv->GetResource( (ID3D11Resource**) &tex);
		tex->GetDesc(&tdesc);
		tex->Release();
		return int2(tdesc.Width, tdesc.Height);
	}
	void TexUpdate(ID3D11ShaderResourceView *srv,void *pix)
	{
		ID3D11Texture2D *tex;
		D3D11_TEXTURE2D_DESC tdesc;
		srv->GetResource((ID3D11Resource**)&tex);
		tex->GetDesc(&tdesc);
		int level = 0;
		dxcontext->UpdateSubresource(tex, level, NULL, (unsigned char *)pix, tdesc.Width * 4, tdesc.Height*tdesc.Width * 4);
		tex->Release();
	}
	void TexUpdate(std::string tname, void *rgba_pixel_data) { return TexUpdate(maps[tname], rgba_pixel_data); }
	ID3D11ShaderResourceView *MakeTex(const unsigned char *buf, int w, int h,bool as_render_target=false) // input image is rgba
	{
		ID3D11ShaderResourceView* textureView;
		ID3D11Texture2D *tex;
		D3D11_TEXTURE2D_DESC tdesc;//, test_tdesc, test_tdescb;
		D3D11_SUBRESOURCE_DATA tbsd;

		void *tmp = NULL;
		tbsd.pSysMem = buf?buf:(tmp=new unsigned char[4*w*h]);
		tbsd.SysMemPitch = w * 4;
		tbsd.SysMemSlicePitch = w*h * 4; // Not needed since this is a 2d texture
		tdesc.Width = w;
		tdesc.Height = h;
		tdesc.MipLevels = 1;
		tdesc.ArraySize = 1;
		tdesc.SampleDesc.Count = 1;
		tdesc.SampleDesc.Quality = 0;
		tdesc.Usage = D3D11_USAGE_DEFAULT;
		tdesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		tdesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | ((as_render_target)?D3D11_BIND_RENDER_TARGET:0);
		tdesc.CPUAccessFlags = 0;
		tdesc.MiscFlags = 0;
		dxdevice->CreateTexture2D(&tdesc, &tbsd, &tex) && VERIFY;
		//tex->GetDesc(&test_tdesc);
		D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc;
		memset(&SRVDesc, 0, sizeof(SRVDesc));
		SRVDesc.Format = tdesc.Format;
		SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		SRVDesc.Texture2D.MipLevels = (!tdesc.MipLevels) ? -1 : tdesc.MipLevels;
		dxdevice->CreateShaderResourceView(tex,&SRVDesc,&textureView)&&VERIFY;
		tex->Release();
		delete tmp;
		return textureView;
	}
	ID3D11ShaderResourceView *MakeTex(const byte4* data, int2 dims) { return MakeTex((unsigned char *)data, dims.x,dims.y); }
	ID3D11ShaderResourceView *MakeTex(const byte3* rgb, int2 dims) 
	{ 
		std::vector<byte4> rgba(product(dims));
		std::transform(rgb, rgb + rgba.size(), rgba.begin(), [](byte3 b) { return byte4(b, 0xFF);});
		return MakeTex(rgba.data(), dims); 
	}
	ID3D11ShaderResourceView *MakeTex(const std::pair<byte3*, int2> &image) { return MakeTex(image.first, image.second); }
	ID3D11DepthStencilView* CreateDepthStencilView(int2 resolution_width_height)
	{
		// need to do all of this even in a simple app that wants the equivalent to glEnable(GL_DEPTH_TEST) 
		ID3D11DepthStencilView* dsv = nullptr;
		ID3D11Texture2D* dxdepthstenciltex = nullptr;   // also creates a texture, but it gets released this and only the 'view' thing is returned
		D3D11_TEXTURE2D_DESC descDepth;
		ZeroMemory(&descDepth, sizeof(descDepth));
		descDepth.Width = resolution_width_height.x;
		descDepth.Height = resolution_width_height.y;
		descDepth.MipLevels = 1;
		descDepth.ArraySize = 1;
		descDepth.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
		descDepth.SampleDesc.Count = 1;
		descDepth.SampleDesc.Quality = 0;
		descDepth.Usage = D3D11_USAGE_DEFAULT;
		descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
		descDepth.CPUAccessFlags = 0;
		descDepth.MiscFlags = 0;
		dxdevice->CreateTexture2D(&descDepth, nullptr, &dxdepthstenciltex) && VERIFY;
		D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
		ZeroMemory(&descDSV, sizeof(descDSV));
		descDSV.Format = descDepth.Format;
		descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
		descDSV.Texture2D.MipSlice = 0;
		dxdevice->CreateDepthStencilView(dxdepthstenciltex, &descDSV, &dsv) && VERIFY;
		dxdepthstenciltex->Release();
		return dsv;
	}
	struct RenderTarget
	{
		int2 dim;
		//ID3D11Texture2D*          tex;
		ID3D11ShaderResourceView* srv;
		ID3D11RenderTargetView*   rendertargetview ;
		ID3D11DepthStencilView*   depthstencilview ;
	}backtarget,practice_target;
	RenderTarget MakeRenderTarget(ID3D11Texture2D *tex,int2 dim)
	{
		ID3D11RenderTargetView*   rendertargetview;
		dxdevice->CreateRenderTargetView(tex, nullptr, &rendertargetview) && VERIFY;  // ("dxdevice->CreateRenderTargetView")
		return{ dim,nullptr, rendertargetview ,CreateDepthStencilView(dim) };
	}
	RenderTarget MakeRenderTarget(int2 dim)
	{
		auto srv = MakeTex(nullptr, dim.x, dim.y,true);
		ID3D11Texture2D *tex;
		srv->GetResource((ID3D11Resource**)&tex);
		auto rt = MakeRenderTarget(tex, dim);
		tex->Release();
		rt.srv = srv;
		return rt;
	}
	void SetUpAndClear(const RenderTarget &rt, float4 clearcolor = { 0,0,0,0 })
	{
		dxcontext->OMSetRenderTargets(1, &rt.rendertargetview, rt.depthstencilview);
		// Setup the viewport
		D3D11_VIEWPORT vp = { 0, 0, (float)rt.dim.x, (float)rt.dim.y, 0.0f, 1.0f };
		dxcontext->RSSetViewports(1, &vp);
		dxcontext->ClearRenderTargetView(rt.rendertargetview, &clearcolor.x);
		dxcontext->ClearDepthStencilView(rt.depthstencilview, D3D11_CLEAR_DEPTH, 1.0f, 0); // Clear depth buffer to 1.0 (max depth)
	}
	HWND CreateDXWindow(const char* title)  // make a d3d11 window
	{
		CreateMSWindow(title, res);
		ShowWindow(hWnd, 1);
		//hDC = GetDC(hWnd);

		HRESULT hr = S_OK;

		RECT rc;
		GetClientRect(hWnd, &rc);
		UINT width = rc.right - rc.left;
		UINT height = rc.bottom - rc.top;

		UINT createDeviceFlags = 0;   // if(debug) createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;

		D3D_FEATURE_LEVEL featureLevels[] = {/*D3D_FEATURE_LEVEL_11_1,*/D3D_FEATURE_LEVEL_11_0,	D3D_FEATURE_LEVEL_10_1,	D3D_FEATURE_LEVEL_10_0 };
		D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;

		D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevels, sizeof(featureLevels) / sizeof(featureLevel),
			D3D11_SDK_VERSION, &dxdevice, &featureLevel, &dxcontext) && VERIFY;  // ("create d3d11 device");

		// Obtain DXGI factory from device (since we used nullptr for pAdapter above)
		IDXGIFactory1* dxgiFactory = nullptr;

		IDXGIDevice* dxgiDevice = nullptr;
		dxdevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice)) && VERIFY;  // ("dxgidevice");
		IDXGIAdapter* adapter = nullptr;
		dxgiDevice->GetAdapter(&adapter) && VERIFY; // "GetAdapter");
		adapter->GetParent(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory)) && VERIFY;  // ("GetParent dxgiFactory");
		dxgiDevice->Release();

		IDXGIFactory2* dxgiFactory2 = nullptr;
		dxgiFactory->QueryInterface(__uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2));
		if (dxgiFactory2) // DirectX 11.1 
		{
			ID3D11DeviceContext1*   dxcontext1 = nullptr;  // note probably should be 'released' at shutdown, but oh well
			IDXGISwapChain1*        dxswapchain1 = nullptr;
			if (dxdevice->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void**>(&dxdevice1)) >= 0)
				(void)dxcontext->QueryInterface(__uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&dxcontext1));

			DXGI_SWAP_CHAIN_DESC1 sd =
			{
				width, height,                          // UINT Width; UINT Height;
				DXGI_FORMAT_R8G8B8A8_UNORM, 			// DXGI_FORMAT Format;
				0, 										// BOOL Stereo;
				{ 1, 0 }, 								// DXGI_SAMPLE_DESC {Count,Quality} SampleDesc; 
				DXGI_USAGE_RENDER_TARGET_OUTPUT, 		// DXGI_USAGE BufferUsage;
				1, 										// UINT BufferCount;
				DXGI_SCALING_STRETCH, 					// DXGI_SCALING Scaling;
				DXGI_SWAP_EFFECT_DISCARD,				// DXGI_SWAP_EFFECT SwapEffect;
				DXGI_ALPHA_MODE_UNSPECIFIED, 			// DXGI_ALPHA_MODE AlphaMode;
				0 										// UINT Flags;
			};
			dxgiFactory2->CreateSwapChainForHwnd(dxdevice, hWnd, &sd, nullptr, nullptr, &dxswapchain1) && VERIFY;  // ("dxgifactory2createswapchain");
			dxswapchain1->QueryInterface(__uuidof(IDXGISwapChain), reinterpret_cast<void**>(&dxswapchain)) && VERIFY;  // ("swapchain QueryInterface");
			dxgiFactory2->Release();
		}
		else // DirectX 11.0     sigh, so much api to using dx 
		{
			DXGI_SWAP_CHAIN_DESC sd;
			ZeroMemory(&sd, sizeof(sd));
			sd.BufferCount = 1;
			sd.BufferDesc.Width = width;
			sd.BufferDesc.Height = height;
			sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			sd.BufferDesc.RefreshRate.Numerator = 60;
			sd.BufferDesc.RefreshRate.Denominator = 1;
			sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
			sd.OutputWindow = hWnd;
			sd.SampleDesc.Count = 1;
			sd.SampleDesc.Quality = 0;
			sd.Windowed = TRUE;

			dxgiFactory->CreateSwapChain(dxdevice, &sd, &dxswapchain) && VERIFY;  // ("dxgiFactory->CreateSwapChain");
		}

		dxgiFactory->Release();

		ID3D11Texture2D* pBackBuffer = nullptr;
		hr = dxswapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBuffer)) && VERIFY;  //("dxswapchain->GetBuffer");
		backtarget = MakeRenderTarget(pBackBuffer, int2(width, height));
		pBackBuffer->Release();
		practice_target = MakeRenderTarget({ 256,256 });
		this->maps["practice"] = practice_target.srv;
		SetUpAndClear(backtarget);

		return hWnd;
	}    


	ID3D11VertexShader*                 pVertexShader   = nullptr;
	ID3D11PixelShader*                  pPixelShader    = nullptr;
	ID3D11InputLayout*                  pVertexLayout   = nullptr;
	ID3D11Buffer*                       pConstantBuffer = nullptr;

	ID3D11ShaderResourceView*           pDMapRV         = nullptr;
	ID3D11ShaderResourceView*           pNMapRV         = nullptr;
	ID3D11SamplerState*                 pSamplerLinear  = nullptr;
	std::map<std::string, ID3D11ShaderResourceView*> maps;
	std::map<std::string, ID3D11PixelShader*> pshaders;
	std::map<std::string, Material>  materials;

	ID3D11DepthStencilState *depth_stencil_state_enabled = nullptr;  
	ID3D11DepthStencilState *depth_stencil_state_disabled= nullptr;

	void CreateShaders()
	{
		D3D11_DEPTH_STENCIL_DESC depthstencildesc = 
		{ 
			TRUE ,
			D3D11_DEPTH_WRITE_MASK_ALL ,
			D3D11_COMPARISON_LESS ,
			FALSE ,
			D3D11_DEFAULT_STENCIL_READ_MASK ,
			D3D11_DEFAULT_STENCIL_WRITE_MASK ,
			{D3D11_STENCIL_OP_KEEP ,D3D11_STENCIL_OP_KEEP,D3D11_STENCIL_OP_KEEP,D3D11_COMPARISON_ALWAYS},
			{D3D11_STENCIL_OP_KEEP ,D3D11_STENCIL_OP_KEEP,D3D11_STENCIL_OP_KEEP,D3D11_COMPARISON_ALWAYS}
		};
		dxdevice->CreateDepthStencilState(&depthstencildesc,&depth_stencil_state_enabled ) && VERIFY;
		depthstencildesc.DepthEnable = false;
		dxdevice->CreateDepthStencilState(&depthstencildesc,&depth_stencil_state_disabled) && VERIFY;

		ID3DBlob* pVSBlob = nullptr;
		ID3DBlob* pErrorBlob = nullptr;
		//hr = CompileShaderFromFile( L"Tutorial02.fx", "VS", "vs_4_0", &pVSBlob );
		D3DCompile(dxshaders, strlen(dxshaders), "vs", NULL, NULL, "VS", "vs_4_0", D3DCOMPILE_DEBUG, 0, &pVSBlob, &pErrorBlob) && VERIFY;
		dxdevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &pVertexShader) && VERIFY;

		D3D11_INPUT_ELEMENT_DESC layout[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT   , 0, OFFSET(Vertex, position   ), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, OFFSET(Vertex, orientation), D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT      , 0, OFFSET(Vertex, texcoord   ), D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};
		UINT numElements = ARRAYSIZE(layout);

		// Create the input layout
		dxdevice->CreateInputLayout(layout,sizeof(layout)/sizeof(*layout), pVSBlob->GetBufferPointer(),pVSBlob->GetBufferSize(), &pVertexLayout) &&VERIFY;
		pVSBlob->Release();
		dxcontext->IASetInputLayout(pVertexLayout);

		for (const char* ps : {"PS", "texonly" ,"rimlit","texlit"})
		{
			ID3DBlob* pPSBlob = nullptr;
			D3DCompile(dxshaders, strlen(dxshaders), "ps", NULL, NULL, ps, "ps_4_0", D3DCOMPILE_DEBUG, 0, &pPSBlob, &pErrorBlob) && VERIFY;  // Compile the pixel shader
			dxdevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &pPixelShader) && VERIFY;  // Create the pixel shader
			pshaders[ps] = pPixelShader;
			pPSBlob->Release();
		}
		ConstantBuffer shaderglobals;
		shaderglobals.projection = MatrixPerspectiveFovAspect(this->ViewAngle, (float)res.x / res.y, 0.05f, 50.0f);
		shaderglobals.hack = { 1, 0, 0, 1 };
		D3D11_BUFFER_DESC bdc =
		{
			sizeof(ConstantBuffer) ,         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_CONSTANT_BUFFER, // UINT        BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA shaderglobals_rc = { &shaderglobals, 0, 0 };
		//dxdevice->CreateBuffer(&bdc, &shaderglobals_rc, &pConstantBuffer) && VERIFY;
		dxdevice->CreateBuffer(&bdc, nullptr, &pConstantBuffer) && VERIFY;

		D3D11_SAMPLER_DESC sampDesc;
		ZeroMemory(&sampDesc, sizeof(sampDesc));
		sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
		sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
		sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
		sampDesc.MinLOD = 0;
		sampDesc.MaxLOD = D3D11_FLOAT32_MAX;
		dxdevice->CreateSamplerState(&sampDesc, &pSamplerLinear) &&VERIFY;

		pDMapRV = MakeTexCheckerboard();
		pNMapRV = MakeTexWav();
		maps["wav"] = pNMapRV;
		maps["checker"] = pDMapRV; 
		// yup, do all the following to specify that "front" faces are ccw.
		D3D11_RASTERIZER_DESC rastdesc = {
			 D3D11_FILL_SOLID, // D3D11_FILL_MODE FillMode;
			 D3D11_CULL_BACK , // D3D11_CULL_MODE CullMode;
			 true            , // BOOL   FrontCounterClockwise;
			 0, 0.0, 0.0,      // INT DepthBias; FLOAT DepthBiasClamp; FLOAT SlopeScaledDepthBias;
			 true,false,false,false,  // BOOL DepthClipEnable, ScissorEnable,  MultisampleEnable,  AntialiasedLineEnable;
		};
		ID3D11RasterizerState *raststate;
		dxdevice->CreateRasterizerState(&rastdesc, &raststate)&&VERIFY;
		dxcontext->RSSetState(raststate);

		D3D11_BLEND_DESC bm;
		memset(&bm, 0, sizeof(bm));
		bm.RenderTarget[0].BlendEnable = true; //  : false;
		bm.RenderTarget[0].BlendOp = bm.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
		bm.RenderTarget[0].SrcBlend = bm.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_SRC_ALPHA;
		bm.RenderTarget[0].DestBlend = bm.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
		bm.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
		dxdevice->CreateBlendState(&bm, &enableblendstate);


		dxcontext->PSSetShader(pPixelShader, nullptr, 0);
		dxcontext->VSSetShader(pVertexShader, nullptr, 0);
		dxcontext->VSSetConstantBuffers(0, 1, &pConstantBuffer);
		dxcontext->PSSetConstantBuffers(0, 1, &pConstantBuffer);
		dxcontext->PSSetSamplers(0, 1, &pSamplerLinear);
		dxcontext->PSSetShaderResources(0, 1, &pDMapRV);
		dxcontext->PSSetShaderResources(1, 1, &pNMapRV);

	}
	ID3D11BlendState        * enableblendstate=NULL;
	void EnableBlend()
	{
		dxcontext->OMSetBlendState(enableblendstate, NULL, 0xffffffff);
	}
	void DestroyDXWindow()
	{
		if(!hWnd)
			return; // already destroyed or otherwise non existent
		hWnd=NULL;
	}

	DXWin(const char *title, int2 res = { 1920,1080 }) : MSWin(title,res) 
	{
		hWnd = CreateDXWindow(title);
		if (hWnd == NULL) throw("failed to create d3d11 window");
		CreateShaders();
	}
	~DXWin()
	{
		DestroyDXWindow();
	}
	void DrawImmediate(const std::vector<Vertex> &verts, const std::vector<int3> &tris)   // worst reimplementation of dx9 style drawprimitiveup() ever
	{
		ID3D11Buffer* pVertexBuffer = nullptr;
		ID3D11Buffer* pIndexBuffer = nullptr;
		D3D11_BUFFER_DESC bdv =
		{
			verts.size() * sizeof(Vertex),         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_VERTEX_BUFFER, // UINT        BindFlags;
			0,0,0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA vertices_rc = { verts.data(), 0, 0 };
		dxdevice->CreateBuffer(&bdv, &vertices_rc, &pVertexBuffer) && VERIFY;

		// Set vertex buffer
		UINT strides[] = { sizeof(Vertex) };
		UINT offsets[] = { 0 };
		dxcontext->IASetVertexBuffers(0, 1, &pVertexBuffer, strides, offsets);

		D3D11_BUFFER_DESC bdt =
		{
			tris.size() * sizeof(int3),          // UINT         ByteWidth;   // full memory byte size of the list eg sizeof(struct)*count
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE  Usage;
			D3D11_BIND_INDEX_BUFFER,  // UINT         BindFlags;
			0, 0, 0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA indices_rc = { tris.data(), 0, 0 };
		dxdevice->CreateBuffer(&bdt, &indices_rc, &pIndexBuffer) && VERIFY;
		dxcontext->IASetIndexBuffer(pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);  // Set index buffer

		dxcontext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);  // Set primitive topology
		dxcontext->DrawIndexed(tris.size()*3, 0, 0);  // triangle count * 3 
		pIndexBuffer->Release();
		pVertexBuffer->Release();
		pIndexBuffer = NULL;
		pVertexBuffer = NULL;
	}
	void DrawPoints(ConstantBuffer cb, const std::vector<Vertex> &verts)   //test
	{
		if (!verts.size())
			return;
		cb.hack  = float4(0.5f, 1, 1, 1);
		cb.meshp = float3(0.0f);
		cb.meshq = float4(0,0,0,1);
		dxcontext->UpdateSubresource(pConstantBuffer, 0, nullptr, &cb, 0, 0);

		ID3D11Buffer* pVertexBuffer = nullptr;
		D3D11_BUFFER_DESC bdv =
		{
			verts.size() * sizeof(Vertex),         // UINT        ByteWidth;
			D3D11_USAGE_DEFAULT,      // D3D11_USAGE Usage;
			D3D11_BIND_VERTEX_BUFFER, // UINT        BindFlags;
			0,0,0  // UINT CPUAccessFlags;UINT MiscFlags;UINT StructureByteStride;
		};
		D3D11_SUBRESOURCE_DATA vertices_rc = { verts.data(), 0, 0 };
		dxdevice->CreateBuffer(&bdv, &vertices_rc, &pVertexBuffer) && VERIFY;

		// Set vertex buffer
		UINT strides[] = { sizeof(Vertex) };
		UINT offsets[] = { 0 };
		dxcontext->IASetVertexBuffers(0, 1, &pVertexBuffer, strides, offsets);

		dxcontext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);  // Set primitive topology
		dxcontext->Draw(verts.size(), 0);  // triangle count * 3 
		pVertexBuffer->Release();
		pVertexBuffer = NULL;
	}
	void DrawPoints(ConstantBuffer cb, const std::vector<float3> &verts)  {DrawPoints(cb,Transform(verts,[](float3 v)->Vertex{return {v,float4(0,0,0,1),{0,0}};}));}

	void SetMaterial(const Material &mat)
	{
			dxcontext->PSSetShaderResources(0, 1, (maps.find(mat.diffusemap) != maps.end()) ? &maps[mat.diffusemap] : &pDMapRV);
			dxcontext->PSSetShader(pshaders[mat.pixelshader], nullptr, 0);
	}
	void SetMaterial(std::string mname = "")
	{
		SetMaterial((materials.find(mname) != materials.end()) ? materials[mname] : Material{ mname, "", "PS" });   // fallback to use material name for diffusemap
	}

	void DrawMeshes(ConstantBuffer cb, const std::vector<Mesh*> &meshes)
	{
		for (auto m : meshes) if(m->verts.size() && m->tris.size())
		{
			if (m->material == "rgbcam")
				dxcontext->OMSetDepthStencilState(depth_stencil_state_disabled, 0);
			SetMaterial(m->material);

			cb.hack  = m->hack;
			cb.meshp = m->pose.position;
			cb.meshq = m->pose.orientation;
			dxcontext->UpdateSubresource(pConstantBuffer, 0, nullptr, &cb, 0, 0);

			DrawImmediate(m->verts, m->tris);
			if (m->material == "rgbcam")
				dxcontext->OMSetDepthStencilState(depth_stencil_state_enabled, 0);
		}
	}
	void RenderScene(const Pose &camera, const std::vector<Mesh*> &meshes,const std::vector<float3> &points)
	{
		SetUpAndClear(backtarget);

		DXWin::ConstantBuffer cb;
		cb.projection = transpose(MatrixPerspectiveFovAspect(ViewAngle*3.14f / 180.0f, aspect_ratio() , 0.05f, 50.0f));
		cb.camerap = camera.position;
		cb.cameraq = camera.orientation;
		DrawMeshes(cb, meshes);
		DrawPoints(cb,points);
		dxswapchain->Present(vsync, 0);
	}
	void RenderStereo(const Pose &camera, const std::vector<Mesh*> &meshes, const std::vector<float3> &points)
	{
		SetUpAndClear(practice_target);
		DXWin::ConstantBuffer cb;
		cb.projection = transpose(MatrixPerspectiveFovAspect(ViewAngle*3.14f / 180.0f, aspect_ratio(), 0.05f, 50.0f));
		cb.camerap = camera.position;
		cb.cameraq = camera.orientation;
		DrawMeshes(cb, meshes);

		SetUpAndClear(backtarget);

		for (int e = 0; e < 2; e++)
		{
			D3D11_VIEWPORT eviewport = { (float)res.x / 2 * e, 0.0f, (float)res.x / 2, (float)res.y, 0.0f, 1.0f };
			dxcontext->RSSetViewports(1, &eviewport);
			DXWin::ConstantBuffer cb;
			cb.projection = transpose(MatrixPerspectiveFovAspect(ViewAngle*3.14f / 180.0f, (float)1 / 1, 0.05f, 50.0f));  // assumes left and right are centered for each 
			cb.cameraq = camera.orientation;
			const float interpupillary_distance = 0.063f;  // assumes meter scale
			cb.camerap = camera.position + qxdir(camera.orientation)*interpupillary_distance * (e ? 0.5f : -0.5f);  
			DrawMeshes(cb, meshes);
		}
		D3D11_VIEWPORT fullviewport = { 0.0f, 0.0f, (float)res.x , (float)res.y, 0.0f, 1.0f };  // res should be same as backtarget.dim
		dxcontext->RSSetViewports(1, &fullviewport);
		dxswapchain->Present(0, 0);
	}

	

};


#endif //SANDBOX_DXWIN_H
