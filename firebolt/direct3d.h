#ifndef DIRECT3D_H
#define DIRECT3D_H

#include <d3dx9.h>
#include <windows.h>

#include "mvector.h"
#include "mesh.h"
#include "camera.h"
#include "gquery.h"

#include <vector>
#include <map>
#include <string>


class Direct3D {

  friend class DMeshClass;

public:

  Direct3D(HWND);
  void Frame(double);

  void SetupStaticBuffers();
  void ReleaseStaticBuffers();

  ray CursorRay(void);

  void DrawText(const std::wstring&, RECT&);
  void DrawTextF(const std::wstring&, RECT&, UINT, D3DXCOLOR&);



  unsigned int getMouseWindowX() { return mouseWindowX; }
  unsigned int getMouseWindowY() { return mouseWindowY; }

  void setMouseWindowX(unsigned int x) { mouseWindowX = x; }
  void setMouseWindowY(unsigned int y) { mouseWindowY = y; }


  unsigned int meshBaseIndex;
  std::vector<Mesh*> static_Meshes;

  D3DXMATRIX viewProjMatrix;


private:

  void streamDMeshs();


  //void streamPolygons();
  void streamStaticMesh(unsigned int i);

  IDirect3D9* interfac;

  IDirect3DDevice9 *device;

  IDirect3DVertexDeclaration9* _decl;
  ID3DXBuffer* errorBuffer ;
  ID3DXBuffer* errorBuffer2;

  Viewport* activeViewport;

  D3DXMATRIX projMatrix;
  D3DXMATRIX viewMatrix;



  unsigned int mouseWindowX, mouseWindowY;
  unsigned int windowWidth, windowHeight;

  IDirect3DVertexBuffer9 *static_vertex_buffer;
  IDirect3DIndexBuffer9 *static_index_buffer;

  std::map<char*, IDirect3DVertexShader9*> vertex_shaders;
  std::map<char*, IDirect3DPixelShader9*> pixel_shaders;

  std::map<char*, ID3DXConstantTable*> constant_tables;
  std::map<char*, D3DXHANDLE> constant_handles;

  std::map<char*, IDirect3DTexture9*> textures;

  std::vector<IDirect3DVertexBuffer9*> matrixVertexBuffers;


  std::vector<ID3DXFont*> fonts;

  ID3DXFont*activeFont;
  D3DXCOLOR fontColor;


};

extern Direct3D *direct3D;

#endif
