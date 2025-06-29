#include "direct3d.h"

#include "dmesh.h"
#include "timer.h"

#include <fstream>


Direct3D *direct3D;


D3DVERTEXELEMENT9 decl[] = {
  {0, 0, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
  {0, 12, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_NORMAL, 0},
  D3DDECL_END() };



ray Direct3D::CursorRay() {

  mVECTOR2 v = activeViewport->inverse((float)getMouseWindowX(), (float)getMouseWindowY());
  D3DXVECTOR3 direction((float)v.x, (float)v.y, 1);
  D3DXVECTOR3 origin(0,0,0);
  D3DXVECTOR4 o;

  D3DXMATRIX mat;
  camera->getViewMatrix(mat);
  D3DXMatrixInverse(&mat,0, &mat);

  D3DXVec3Transform(&o, &origin, &mat);

  origin.x = o.x;
  origin.y = o.y;
  origin.z = o.z;

  D3DXVec3TransformNormal(&direction, &direction, &mat);

  return ray(origin, direction);

}


Direct3D::Direct3D(HWND hWnd) : meshBaseIndex(0) {

  RECT rc;
  GetClientRect(hWnd, &rc );

  windowHeight = rc.bottom - rc.top;
  windowWidth = rc.right - rc.left;


  interfac = Direct3DCreate9(D3D_SDK_VERSION) ;

  D3DPRESENT_PARAMETERS d3dpp;
  d3dpp.BackBufferWidth            = windowWidth ;
  d3dpp.BackBufferHeight           = windowHeight;
  d3dpp.BackBufferFormat           = D3DFMT_A8R8G8B8;
  d3dpp.BackBufferCount            = 1;
  d3dpp.MultiSampleType            = D3DMULTISAMPLE_NONE;
  d3dpp.MultiSampleQuality         = 0;
  d3dpp.SwapEffect                 = D3DSWAPEFFECT_DISCARD;
  d3dpp.hDeviceWindow              = hWnd;
  d3dpp.Windowed                   = true;
  d3dpp.EnableAutoDepthStencil     = true;
  d3dpp.AutoDepthStencilFormat     = D3DFMT_D24S8;
  d3dpp.Flags                      = 0;
  d3dpp.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;
  d3dpp.PresentationInterval       = D3DPRESENT_INTERVAL_ONE;

  interfac->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &d3dpp, &device );


  //SETUP CAMERA
  camera = new Camera(D3DX_PI * 0.3, (float)windowWidth / (float)windowHeight, 1.0f, 1e6f);
  camera->getProjectionMatrix(projMatrix);

  activeViewport = new Viewport(0, 0, windowWidth, windowHeight);
  activeViewport->activeCamera = camera;

  device->CreateVertexDeclaration(decl, &_decl);


  ID3DXBuffer* helper_buffer =0;

  IDirect3DVertexShader9* helper_shader=0;
  IDirect3DPixelShader9* helper_shader2=0;
  D3DXCompileShaderFromFile(L"vs_dot.txt", 0, 0, "main", "vs_2_0", 0, &helper_buffer, &errorBuffer, &constant_tables["vs_dot"]);
  device->CreateVertexShader((DWORD*)helper_buffer->GetBufferPointer(), &helper_shader);
  vertex_shaders["vs_dot"] = helper_shader;

  D3DXCompileShaderFromFile(L"ps_dot.txt", 0, 0, "main", "ps_2_0", 0,      &helper_buffer, &errorBuffer2, &constant_tables["ps_dot"]);
  device->CreatePixelShader((DWORD*)helper_buffer->GetBufferPointer(), &helper_shader2);
  pixel_shaders["ps_dot"] = helper_shader2;

  constant_handles["vs_dot_light_position"] = constant_tables["vs_dot"]->GetConstantByName(0, "lightDirection");
  constant_handles["vs_dot_worldViewProj"] = constant_tables["vs_dot"]->GetConstantByName(0, "worldViewProj");
  constant_handles["vs_dot_color"] = constant_tables["vs_dot"]->GetConstantByName(0, "color");

  device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE );

  D3DXFONT_DESC desc;
  ID3DXFont *def =0;

    desc.Height =20;
    desc.Width =10;
    desc.Weight =30;
    desc.MipLevels = 0;
    desc.Italic = 0;
    desc.CharSet =DEFAULT_CHARSET;
    desc.OutputPrecision =OUT_TT_ONLY_PRECIS;
    desc.Quality = ANTIALIASED_QUALITY;
    desc.PitchAndFamily = FF_MODERN;

  WCHAR b[32] = L"Times New Roman";
  memcpy(desc.FaceName, b, sizeof(WCHAR[32]));

  D3DXCreateFontIndirect(device, &desc, &def);
  fonts.push_back(def);
  activeFont = fonts[0];
  D3DXCOLOR color;
  color.r = color.g = color.b = color.a =0.0f;
  color.g = color.a = 1.0f;
  fontColor = color;

  device->CreateVertexBuffer(10000*sizeof(vertex), D3DUSAGE_DYNAMIC,  0, D3DPOOL_DEFAULT, &static_vertex_buffer, 0);

  device->CreateIndexBuffer(10000*sizeof(face), D3DUSAGE_DYNAMIC, D3DFMT_INDEX16, D3DPOOL_DEFAULT, &static_index_buffer, 0);

}


void Direct3D::SetupStaticBuffers() {

  unsigned int total_vertex_count(0), total_face_count(0);

  std::vector<Mesh*>::iterator it;
  for(it = static_Meshes.begin(); it!= static_Meshes.end(); it++) {

    total_vertex_count +=(*it)->vertex_count;
    total_face_count +=(*it)->face_count;

  }

  VOID * Data =0;
  VOID * Data2 =0;

  static_vertex_buffer->Lock(0,0,&Data,D3DLOCK_DISCARD);
  static_index_buffer->Lock(0,0,&Data2,D3DLOCK_DISCARD);

  int voffset=0;
  int ioffset=0;

  for(it = static_Meshes.begin(); it!=static_Meshes.end(); it++) {

    (*it)->voffset = voffset;
    memcpy( ((void*)(((vertex*)Data)+ voffset)), (*it)->vertices, (*it)->vertex_count*sizeof(vertex));
    voffset+=(*it)->vertex_count;

    (*it)->ioffset = ioffset;
    memcpy(((void*)(((unsigned short*)Data2)+ ioffset)), (*it)->faces, (*it)->face_count*sizeof(face));
    ioffset+=(*it)->face_count*3;

   }

  static_vertex_buffer->Unlock();
  static_index_buffer->Unlock();

}

void Direct3D::ReleaseStaticBuffers() {

  meshBaseIndex =0;

  std::vector<Mesh*>::iterator it;
  for(it = static_Meshes.begin(); it != static_Meshes.end(); it++)
    delete (*it);
  static_Meshes.clear();
  matrixVertexBuffers.clear();

}

void Direct3D::streamDMeshs() {

  device->SetVertexDeclaration(_decl);
  device->SetIndices(static_index_buffer);
  device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CCW);


  device->SetVertexShader(vertex_shaders["vs_dot"]);
  device->SetPixelShader(pixel_shaders["ps_dot"]);

  D3DXVECTOR4 light_vector = camera->getViewDirection();

  light_vector.w = 0.0;
  D3DXVec4Normalize(&light_vector, &light_vector);

  device->SetStreamSource(0, static_vertex_buffer, 0, sizeof(vertex));

  dmeshManager.streamDMeshClasses();

}

void Direct3D::streamStaticMesh(unsigned int i) {

  device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, static_Meshes[i]->voffset, 0,  0, static_Meshes[i]->ioffset, static_Meshes[i]->face_count);

}


void Direct3D::Frame(double time_delta) {

  camera->getProjectionMatrix(projMatrix);
  camera->getViewMatrix(viewMatrix);
  viewProjMatrix = viewMatrix * projMatrix;

  device->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0x00000000, 1.0f, 0);

  device->BeginScene();


  std::wstring text(L"2-7 - load scences \n p - pause/play \n w,s,a,d - movement \n RMB - look around");
  RECT r; r.bottom = 100;  r.left = 0;  r.right = 200;  r.top = 0;
  DrawText(text, r);


  streamDMeshs();

  device->EndScene();
  device->Present(0, 0, 0, 0);

}

void Direct3D::DrawText(const std::wstring &text, RECT &r) {

  activeFont->DrawTextW(NULL, text.c_str(), text.size(), &r,DT_CENTER | DT_VCENTER ,fontColor);

}

void  Direct3D::DrawTextF(const std::wstring &text, RECT &rect, UINT format, D3DXCOLOR &color) {

  activeFont->DrawTextW(NULL, text.c_str(), text.size(), &rect, format, color);

}
