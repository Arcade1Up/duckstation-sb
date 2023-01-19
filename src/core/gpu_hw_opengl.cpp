#include "gpu_hw_opengl.h"
#include "common/assert.h"
#include "common/log.h"
#include "common/state_wrapper.h"
#include "common/timer.h"
#include "gpu_hw_shadergen.h"
#include "host_display.h"
#include "shader_cache_version.h"
#include "system.h"
#include "texture_replacements.h"
Log_SetChannel(GPU_HW_OpenGL);

GPU_HW_OpenGL::GPU_HW_OpenGL() : GPU_HW() {}

GPU_HW_OpenGL::~GPU_HW_OpenGL()
{
  // Destroy objects which don't have destructors to clean them up
  if (m_vram_fbo_id != 0)
    glDeleteFramebuffers(1, &m_vram_fbo_id);
  if (m_vao_id != 0)
    glDeleteVertexArrays(1, &m_vao_id);
  if (m_attributeless_vao_id != 0)
    glDeleteVertexArrays(1, &m_attributeless_vao_id);
  if (m_texture_buffer_r16ui_texture != 0)
    glDeleteTextures(1, &m_texture_buffer_r16ui_texture);

  if (m_host_display)
  {
    m_host_display->ClearDisplayTexture();
    ResetGraphicsAPIState();
  }

  // One of our programs might've been bound.
  GL::Program::ResetLastProgram();
  glUseProgram(0);
}

GPURenderer GPU_HW_OpenGL::GetRendererType() const
{
  return GPURenderer::HardwareOpenGL;
}

bool GPU_HW_OpenGL::Initialize(HostDisplay* host_display)
{
  if (host_display->GetRenderAPI() != HostDisplay::RenderAPI::OpenGL &&
      host_display->GetRenderAPI() != HostDisplay::RenderAPI::OpenGLES)
  {
    Log_ErrorPrintf("Host render API type is incompatible");
    return false;
  }

  const bool opengl_is_available =
    ((host_display->GetRenderAPI() == HostDisplay::RenderAPI::OpenGL &&
      (GLAD_GL_VERSION_3_0 || GLAD_GL_ARB_uniform_buffer_object)) ||
     (host_display->GetRenderAPI() == HostDisplay::RenderAPI::OpenGLES && GLAD_GL_ES_VERSION_3_0));
  if (!opengl_is_available)
  {
    g_host_interface->AddOSDMessage(
      g_host_interface->TranslateStdString("OSDMessage", "OpenGL renderer unavailable, your driver or hardware is not "
                                                         "recent enough. OpenGL 3.1 or OpenGL ES 3.0 is required."),
      20.0f);
    return false;
  }

  SetCapabilities(host_display);

  if (!GPU_HW::Initialize(host_display))
    return false;

  if (!CreateFramebuffer())
  {
    Log_ErrorPrintf("Failed to create framebuffer");
    return false;
  }

  if (!CreateVertexBuffer())
  {
    Log_ErrorPrintf("Failed to create vertex buffer");
    return false;
  }

  if (!CreateUniformBuffer())
  {
    Log_ErrorPrintf("Failed to create uniform buffer");
    return false;
  }

  if (!CreateTextureBuffer())
  {
    Log_ErrorPrintf("Failed to create texture buffer");
    return false;
  }

  if (!CompilePrograms())
  {
    Log_ErrorPrintf("Failed to compile programs");
    return false;
  }

  RestoreGraphicsAPIState();
  return true;
}

void GPU_HW_OpenGL::Reset(bool clear_vram)
{
  GPU_HW::Reset(clear_vram);

  if (clear_vram)
    ClearFramebuffer();
}

bool GPU_HW_OpenGL::DoState(StateWrapper& sw, HostDisplayTexture** host_texture, bool update_display)
{
  if (host_texture)
  {
    HostDisplayTexture* tex = *host_texture;
    if (sw.IsReading())
    {
      if (tex->GetWidth() != m_vram_texture.GetWidth() || tex->GetHeight() != m_vram_texture.GetHeight() ||
          tex->GetSamples() != m_vram_texture.GetSamples())
      {
        return false;
      }

      CopyFramebufferForState(
        m_vram_texture.GetGLTarget(), static_cast<GLuint>(reinterpret_cast<uintptr_t>(tex->GetHandle())), 0, 0, 0,
        m_vram_texture.GetGLId(), m_vram_fbo_id, 0, 0, m_vram_texture.GetWidth(), m_vram_texture.GetHeight());
    }
    else
    {
      if (!tex || tex->GetWidth() != m_vram_texture.GetWidth() || tex->GetHeight() != m_vram_texture.GetHeight() ||
          tex->GetSamples() != m_vram_texture.GetSamples())
      {
        delete tex;

        tex = m_host_display
                ->CreateTexture(m_vram_texture.GetWidth(), m_vram_texture.GetHeight(), 1, 1,
                                m_vram_texture.GetSamples(), HostDisplayPixelFormat::RGBA8, nullptr, 0, false)
                .release();
        *host_texture = tex;
        if (!tex)
          return false;
      }

      CopyFramebufferForState(m_vram_texture.GetGLTarget(), m_vram_texture.GetGLId(), m_vram_fbo_id, 0, 0,
                              static_cast<GLuint>(reinterpret_cast<uintptr_t>(tex->GetHandle())), 0, 0, 0,
                              m_vram_texture.GetWidth(), m_vram_texture.GetHeight());
    }
  }

  return GPU_HW::DoState(sw, host_texture, update_display);
}

void GPU_HW_OpenGL::CopyFramebufferForState(GLenum target, GLuint src_texture, u32 src_fbo, u32 src_x, u32 src_y,
                                            GLuint dst_texture, u32 dst_fbo, u32 dst_x, u32 dst_y, u32 width,
                                            u32 height)
{
  if (target != GL_TEXTURE_2D && GLAD_GL_VERSION_4_3)
  {
    glCopyImageSubData(src_texture, target, 0, src_x, src_y, 0, dst_texture, target, 0, dst_x, dst_y, 0, width, height,
                       1);
  }
  else if (target != GL_TEXTURE_2D && GLAD_GL_EXT_copy_image)
  {
    glCopyImageSubDataEXT(src_texture, target, 0, src_x, src_y, 0, dst_texture, target, 0, dst_x, dst_y, 0, width,
                          height, 1);
  }
  else if (target != GL_TEXTURE_2D && GLAD_GL_OES_copy_image)
  {
    glCopyImageSubDataOES(src_texture, target, 0, src_x, src_y, 0, dst_texture, target, 0, dst_x, dst_y, 0, width,
                          height, 1);
  }
  else
  {
    if (src_fbo == 0)
    {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, m_state_copy_fbo_id);
      glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, target, src_texture, 0);
    }
    else
    {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, src_fbo);
    }

    if (dst_fbo == 0)
    {
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_state_copy_fbo_id);
      glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, target, dst_texture, 0);
    }
    else
    {
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dst_fbo);
    }

    glDisable(GL_SCISSOR_TEST);
    glBlitFramebuffer(src_x, src_y, src_x + width, src_y + height, dst_x, dst_y, dst_x + width, dst_y + height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glEnable(GL_SCISSOR_TEST);

    if (src_fbo == 0)
    {
      glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    }
    else if (dst_fbo == 0)
    {
      glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    }

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
  }
}

void GPU_HW_OpenGL::ResetGraphicsAPIState()
{
  GPU_HW::ResetGraphicsAPIState();

  glEnable(GL_CULL_FACE);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  m_uniform_stream_buffer->Unbind();
}

void GPU_HW_OpenGL::RestoreGraphicsAPIState()
{
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
  glViewport(0, 0, m_vram_texture.GetWidth(), m_vram_texture.GetHeight());

  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_SCISSOR_TEST);
  glDepthMask(GL_TRUE);
  glBindVertexArray(m_vao_id);
  m_uniform_stream_buffer->Bind();
  m_vram_read_texture.Bind();
  SetBlendMode();
  m_current_depth_test = 0;
  SetDepthFunc();
  SetScissorFromDrawingArea();
  m_batch_ubo_dirty = true;
}

void GPU_HW_OpenGL::UpdateSettings()
{
  GPU_HW::UpdateSettings();

  bool framebuffer_changed, shaders_changed;
  UpdateHWSettings(&framebuffer_changed, &shaders_changed);

  if (framebuffer_changed)
  {
    RestoreGraphicsAPIState();
    ReadVRAM(0, 0, VRAM_WIDTH, VRAM_HEIGHT);
    ResetGraphicsAPIState();
    m_host_display->ClearDisplayTexture();
    CreateFramebuffer();
  }
  if (shaders_changed)
    CompilePrograms();

  if (framebuffer_changed)
  {
    RestoreGraphicsAPIState();
    UpdateVRAM(0, 0, VRAM_WIDTH, VRAM_HEIGHT, m_vram_ptr, false, false);
    UpdateDepthBufferFromMaskBit();
    UpdateDisplay();
    ResetGraphicsAPIState();
  }
}

void GPU_HW_OpenGL::MapBatchVertexPointer(u32 required_vertices)
{
  DebugAssert(!m_batch_start_vertex_ptr);

  const GL::StreamBuffer::MappingResult res =
    m_vertex_stream_buffer->Map(sizeof(BatchVertex), required_vertices * sizeof(BatchVertex));

  m_batch_start_vertex_ptr = static_cast<BatchVertex*>(res.pointer);
  m_batch_current_vertex_ptr = m_batch_start_vertex_ptr;
  m_batch_end_vertex_ptr = m_batch_start_vertex_ptr + res.space_aligned;
  m_batch_base_vertex = res.index_aligned;
}

void GPU_HW_OpenGL::UnmapBatchVertexPointer(u32 used_vertices)
{
  DebugAssert(m_batch_start_vertex_ptr);

  m_vertex_stream_buffer->Unmap(used_vertices * sizeof(BatchVertex));
  m_vertex_stream_buffer->Bind();
  m_batch_start_vertex_ptr = nullptr;
  m_batch_end_vertex_ptr = nullptr;
  m_batch_current_vertex_ptr = nullptr;
}

std::tuple<s32, s32> GPU_HW_OpenGL::ConvertToFramebufferCoordinates(s32 x, s32 y)
{
  return std::make_tuple(x, static_cast<s32>(static_cast<s32>(VRAM_HEIGHT) - y));
}

void GPU_HW_OpenGL::SetCapabilities(HostDisplay* host_display)
{
  GLint max_texture_size = VRAM_WIDTH;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
  Log_InfoPrintf("Max texture size: %dx%d", max_texture_size, max_texture_size);
  m_max_resolution_scale = static_cast<u32>(max_texture_size / VRAM_WIDTH);

  m_max_multisamples = 1;
  if (GLAD_GL_ARB_texture_storage || GLAD_GL_ES_VERSION_3_2)
  {
    glGetIntegerv(GL_MAX_SAMPLES, reinterpret_cast<GLint*>(&m_max_multisamples));
    if (m_max_multisamples == 0)
      m_max_multisamples = 1;
  }

  m_supports_per_sample_shading = GLAD_GL_VERSION_4_0 || GLAD_GL_ES_VERSION_3_2 || GLAD_GL_ARB_sample_shading;
  Log_InfoPrintf("Per-sample shading: %s", m_supports_per_sample_shading ? "supported" : "not supported");
  Log_InfoPrintf("Max multisamples: %u", m_max_multisamples);

  glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, reinterpret_cast<GLint*>(&m_uniform_buffer_alignment));
  Log_InfoPrintf("Uniform buffer offset alignment: %u", m_uniform_buffer_alignment);

  if (!GLAD_GL_VERSION_4_3 && !GLAD_GL_EXT_copy_image && !GLAD_GL_ES_VERSION_3_2 && !GLAD_GL_OES_copy_image)
    Log_WarningPrintf("GL_EXT/OES_copy_image missing, this may affect performance.");

#ifdef __APPLE__
  // Partial texture buffer uploads appear to be broken in macOS's OpenGL driver.
  m_use_texture_buffer_for_vram_writes = false;
#else
  m_use_texture_buffer_for_vram_writes = (GLAD_GL_VERSION_3_1 || GLAD_GL_ES_VERSION_3_2);
#endif
  m_texture_stream_buffer_size = VRAM_UPDATE_TEXTURE_BUFFER_SIZE;
  if (m_use_texture_buffer_for_vram_writes)
  {
    GLint max_texel_buffer_size;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, reinterpret_cast<GLint*>(&max_texel_buffer_size));
    Log_InfoPrintf("Max texel buffer size: %u", max_texel_buffer_size);
    if (max_texel_buffer_size < static_cast<int>(VRAM_WIDTH * VRAM_HEIGHT))
    {
      Log_WarningPrintf("Maximum texture buffer size is less than VRAM size, not using texel buffers.");
      m_use_texture_buffer_for_vram_writes = false;
    }
    else
    {
      m_texture_stream_buffer_size =
        std::min<u32>(VRAM_UPDATE_TEXTURE_BUFFER_SIZE, static_cast<u32>(max_texel_buffer_size) * sizeof(u16));
    }
  }

  if (!m_use_texture_buffer_for_vram_writes)
  {
    // Try SSBOs.
    GLint max_fragment_storage_blocks = 0;
    GLint64 max_ssbo_size = 0;
    if (GLAD_GL_VERSION_4_3 || GLAD_GL_ES_VERSION_3_1 || GLAD_GL_ARB_shader_storage_buffer_object)
    {
      glGetIntegerv(GL_MAX_FRAGMENT_SHADER_STORAGE_BLOCKS, &max_fragment_storage_blocks);
      glGetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &max_ssbo_size);
    }

    Log_InfoPrintf("Max fragment shader storage blocks: %d", max_fragment_storage_blocks);
    Log_InfoPrintf("Max shader storage buffer size: %" PRId64, max_ssbo_size);
    m_use_ssbo_for_vram_writes = (max_fragment_storage_blocks > 0 &&
                                  max_ssbo_size >= static_cast<GLint64>(VRAM_WIDTH * VRAM_HEIGHT * sizeof(u16)));
    if (m_use_ssbo_for_vram_writes)
    {
      Log_InfoPrintf("Using shader storage buffers for VRAM writes.");
      m_texture_stream_buffer_size =
        static_cast<u32>(std::min<u64>(VRAM_UPDATE_TEXTURE_BUFFER_SIZE, static_cast<u64>(max_ssbo_size)));
    }
    else
    {
      Log_WarningPrintf("Texture buffers and SSBOs are not supported, VRAM writes will be slower and multisampling "
                        "will be unavailable.");
      m_max_multisamples = 1;
      m_supports_per_sample_shading = false;
    }
  }

  int max_dual_source_draw_buffers = 0;
  glGetIntegerv(GL_MAX_DUAL_SOURCE_DRAW_BUFFERS, &max_dual_source_draw_buffers);
  m_supports_dual_source_blend =
    (max_dual_source_draw_buffers > 0) &&
    (GLAD_GL_VERSION_3_3 || GLAD_GL_ARB_blend_func_extended || GLAD_GL_EXT_blend_func_extended);

  // adaptive smoothing would require texture views, which aren't in GLES.
  m_supports_adaptive_downsampling = false;
}

bool GPU_HW_OpenGL::CreateFramebuffer()
{
  // scale vram size to internal resolution
  const u32 texture_width = VRAM_WIDTH * m_resolution_scale;
  const u32 texture_height = VRAM_HEIGHT * m_resolution_scale;
  const u32 multisamples = m_multisamples;

  if (!m_vram_texture.Create(texture_width, texture_height, multisamples, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, nullptr,
                             false, true) ||
      !m_vram_depth_texture.Create(texture_width, texture_height, multisamples, GL_DEPTH_COMPONENT16,
                                   GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, nullptr, false) ||
      !m_vram_read_texture.Create(texture_width, texture_height, 1, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, nullptr, false,
                                  true) ||
      !m_vram_read_texture.CreateFramebuffer() ||
      !m_vram_encoding_texture.Create(VRAM_WIDTH, VRAM_HEIGHT, 1, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, nullptr,
                                      false) ||
      !m_vram_encoding_texture.CreateFramebuffer() ||
      !m_display_texture.Create(GPU_MAX_DISPLAY_WIDTH * m_resolution_scale, GPU_MAX_DISPLAY_HEIGHT * m_resolution_scale,
                                1, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, nullptr, false) ||
      !m_display_texture.CreateFramebuffer())
  {
    return false;
  }

  if (m_vram_fbo_id == 0)
    glGenFramebuffers(1, &m_vram_fbo_id);

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
  glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_vram_texture.GetGLTarget(),
                         m_vram_texture.GetGLId(), 0);
  glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_vram_depth_texture.GetGLTarget(),
                         m_vram_depth_texture.GetGLId(), 0);
  Assert(glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

  if (m_downsample_mode == GPUDownsampleMode::Box)
  {
    if (!m_downsample_texture.Create(VRAM_WIDTH, VRAM_HEIGHT, 1, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE) ||
        !m_downsample_texture.CreateFramebuffer())
    {
      return false;
    }
  }

  if (m_state_copy_fbo_id == 0)
    glGenFramebuffers(1, &m_state_copy_fbo_id);

  SetFullVRAMDirtyRectangle();
  return true;
}

void GPU_HW_OpenGL::ClearFramebuffer()
{
  const float depth_clear_value = m_pgxp_depth_buffer ? 1.0f : 0.0f;

  glDisable(GL_SCISSOR_TEST);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  IsGLES() ? glClearDepthf(depth_clear_value) : glClearDepth(depth_clear_value);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_SCISSOR_TEST);
  m_last_depth_z = 1.0f;
  SetFullVRAMDirtyRectangle();
}

bool GPU_HW_OpenGL::CreateVertexBuffer()
{
  m_vertex_stream_buffer = GL::StreamBuffer::Create(GL_ARRAY_BUFFER, VERTEX_BUFFER_SIZE);
  if (!m_vertex_stream_buffer)
    return false;

  m_vertex_stream_buffer->Bind();

  glGenVertexArrays(1, &m_vao_id);
  glBindVertexArray(m_vao_id);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);
  glEnableVertexAttribArray(3);
  glEnableVertexAttribArray(4);
  glVertexAttribPointer(0, 4, GL_FLOAT, false, sizeof(BatchVertex), reinterpret_cast<void*>(offsetof(BatchVertex, x)));
  glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, true, sizeof(BatchVertex),
                        reinterpret_cast<void*>(offsetof(BatchVertex, color)));
  glVertexAttribIPointer(2, 1, GL_UNSIGNED_INT, sizeof(BatchVertex), reinterpret_cast<void*>(offsetof(BatchVertex, u)));
  glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, sizeof(BatchVertex),
                         reinterpret_cast<void*>(offsetof(BatchVertex, texpage)));
  glVertexAttribPointer(4, 4, GL_UNSIGNED_BYTE, true, sizeof(BatchVertex),
                        reinterpret_cast<void*>(offsetof(BatchVertex, uv_limits)));
  glBindVertexArray(0);

  glGenVertexArrays(1, &m_attributeless_vao_id);
  return true;
}

bool GPU_HW_OpenGL::CreateUniformBuffer()
{
  m_uniform_stream_buffer = GL::StreamBuffer::Create(GL_UNIFORM_BUFFER, UNIFORM_BUFFER_SIZE);
  if (!m_uniform_stream_buffer)
    return false;

  return true;
}

bool GPU_HW_OpenGL::CreateTextureBuffer()
{
  const GLenum target =
    (m_use_ssbo_for_vram_writes ? GL_SHADER_STORAGE_BUFFER :
                                  (m_use_texture_buffer_for_vram_writes ? GL_TEXTURE_BUFFER : GL_PIXEL_UNPACK_BUFFER));
  m_texture_stream_buffer = GL::StreamBuffer::Create(target, m_texture_stream_buffer_size);
  if (!m_texture_stream_buffer)
    return false;

  if (m_use_texture_buffer_for_vram_writes)
  {
    glGenTextures(1, &m_texture_buffer_r16ui_texture);
    glBindTexture(GL_TEXTURE_BUFFER, m_texture_buffer_r16ui_texture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R16UI, m_texture_stream_buffer->GetGLBufferId());
  }

  m_texture_stream_buffer->Unbind();
  return true;
}

bool GPU_HW_OpenGL::CompilePrograms()
{
  GL::ShaderCache shader_cache;
  shader_cache.Open(IsGLES(), g_host_interface->GetShaderCacheBasePath(), SHADER_CACHE_VERSION);

  const bool use_binding_layout = GPU_HW_ShaderGen::UseGLSLBindingLayout();
  GPU_HW_ShaderGen shadergen(m_host_display->GetRenderAPI(), m_resolution_scale, m_multisamples, m_per_sample_shading,
                             m_true_color, m_scaled_dithering, m_texture_filtering, m_using_uv_limits,
                             m_pgxp_depth_buffer, m_supports_dual_source_blend);

  ShaderCompileProgressTracker progress("Compiling Programs", (4 * 9 * 2 * 2) + (2 * 3) + (2 * 2) + 1 + 1 + 1 + 1 + 1);

  for (u32 render_mode = 0; render_mode < 4; render_mode++)
  {
    for (u32 texture_mode = 0; texture_mode < 9; texture_mode++)
    {
      for (u8 dithering = 0; dithering < 2; dithering++)
      {
        for (u8 interlacing = 0; interlacing < 2; interlacing++)
        {
          const bool textured = (static_cast<GPUTextureMode>(texture_mode) != GPUTextureMode::Disabled);
          const std::string batch_vs = shadergen.GenerateBatchVertexShader(textured);
          const std::string fs = shadergen.GenerateBatchFragmentShader(
            static_cast<BatchRenderMode>(render_mode), static_cast<GPUTextureMode>(texture_mode),
            ConvertToBoolUnchecked(dithering), ConvertToBoolUnchecked(interlacing));

          const auto link_callback = [this, textured, use_binding_layout](GL::Program& prog) {
            if (!use_binding_layout)
            {
              prog.BindAttribute(0, "a_pos");
              prog.BindAttribute(1, "a_col0");
              if (textured)
              {
                prog.BindAttribute(2, "a_texcoord");
                prog.BindAttribute(3, "a_texpage");
                prog.BindAttribute(4, "a_uv_limits");
              }

              if (!IsGLES() || m_supports_dual_source_blend)
              {
                if (m_supports_dual_source_blend)
                {
                  prog.BindFragDataIndexed(0, "o_col0");
                  prog.BindFragDataIndexed(1, "o_col1");
                }
                else
                {
                  prog.BindFragData(0, "o_col0");
                }
              }
            }
          };

          std::optional<GL::Program> prog = shader_cache.GetProgram(batch_vs, {}, fs, link_callback);
          if (!prog)
            return false;

          if (!use_binding_layout)
          {
            prog->BindUniformBlock("UBOBlock", 1);
            if (textured)
            {
              prog->Bind();
              prog->Uniform1i("samp0", 0);
            }
          }

          m_render_programs[render_mode][texture_mode][dithering][interlacing] = std::move(*prog);

          progress.Increment();
        }
      }
    }
  }

  for (u8 depth_24bit = 0; depth_24bit < 2; depth_24bit++)
  {
    for (u8 interlaced = 0; interlaced < 3; interlaced++)
    {
      const std::string vs = shadergen.GenerateScreenQuadVertexShader();
      const std::string fs = shadergen.GenerateDisplayFragmentShader(
        ConvertToBoolUnchecked(depth_24bit), static_cast<InterlacedRenderMode>(interlaced), m_chroma_smoothing);

      std::optional<GL::Program> prog =
        shader_cache.GetProgram(vs, {}, fs, [this, use_binding_layout](GL::Program& prog) {
          if (!IsGLES() && !use_binding_layout)
            prog.BindFragData(0, "o_col0");
        });
      if (!prog)
        return false;

      if (!use_binding_layout)
      {
        prog->BindUniformBlock("UBOBlock", 1);
        prog->Bind();
        prog->Uniform1i("samp0", 0);
      }
      m_display_programs[depth_24bit][interlaced] = std::move(*prog);
      progress.Increment();
    }
  }

  for (u8 wrapped = 0; wrapped < 2; wrapped++)
  {
    for (u8 interlaced = 0; interlaced < 2; interlaced++)
    {
      std::optional<GL::Program> prog = shader_cache.GetProgram(
        shadergen.GenerateScreenQuadVertexShader(), {},
        shadergen.GenerateVRAMFillFragmentShader(ConvertToBoolUnchecked(wrapped), ConvertToBoolUnchecked(interlaced)),
        [this, use_binding_layout](GL::Program& prog) {
          if (!IsGLES() && !use_binding_layout)
            prog.BindFragData(0, "o_col0");
        });
      if (!prog)
        return false;

      if (!use_binding_layout)
        prog->BindUniformBlock("UBOBlock", 1);

      m_vram_fill_programs[wrapped][interlaced] = std::move(*prog);
      progress.Increment();
    }
  }

  std::optional<GL::Program> prog =
    shader_cache.GetProgram(shadergen.GenerateScreenQuadVertexShader(), {}, shadergen.GenerateVRAMReadFragmentShader(),
                            [this, use_binding_layout](GL::Program& prog) {
                              if (!IsGLES() && !use_binding_layout)
                                prog.BindFragData(0, "o_col0");
                            });
  if (!prog)
    return false;

  if (!use_binding_layout)
  {
    prog->BindUniformBlock("UBOBlock", 1);
    prog->Bind();
    prog->Uniform1i("samp0", 0);
  }
  m_vram_read_program = std::move(*prog);
  progress.Increment();

  prog =
    shader_cache.GetProgram(shadergen.GenerateScreenQuadVertexShader(), {}, shadergen.GenerateVRAMCopyFragmentShader(),
                            [this, use_binding_layout](GL::Program& prog) {
                              if (!IsGLES() && !use_binding_layout)
                                prog.BindFragData(0, "o_col0");
                            });
  if (!prog)
    return false;

  if (!use_binding_layout)
  {
    prog->BindUniformBlock("UBOBlock", 1);
    prog->Bind();
    prog->Uniform1i("samp0", 0);
  }
  m_vram_copy_program = std::move(*prog);
  progress.Increment();

  prog = shader_cache.GetProgram(shadergen.GenerateScreenQuadVertexShader(), {},
                                 shadergen.GenerateVRAMUpdateDepthFragmentShader());
  if (!prog)
    return false;

  prog->Bind();
  prog->Uniform1i("samp0", 0);
  m_vram_update_depth_program = std::move(*prog);
  progress.Increment();

  if (m_use_texture_buffer_for_vram_writes || m_use_ssbo_for_vram_writes)
  {
    prog = shader_cache.GetProgram(shadergen.GenerateScreenQuadVertexShader(), {},
                                   shadergen.GenerateVRAMWriteFragmentShader(m_use_ssbo_for_vram_writes),
                                   [this, use_binding_layout](GL::Program& prog) {
                                     if (!IsGLES() && !use_binding_layout)
                                       prog.BindFragData(0, "o_col0");
                                   });
    if (!prog)
      return false;

    if (!use_binding_layout)
    {
      prog->BindUniformBlock("UBOBlock", 1);
      prog->Bind();
      prog->Uniform1i("samp0", 0);
    }
    m_vram_write_program = std::move(*prog);
  }

  progress.Increment();

  if (m_downsample_mode == GPUDownsampleMode::Box)
  {
    prog = shader_cache.GetProgram(shadergen.GenerateScreenQuadVertexShader(), {},
                                   shadergen.GenerateBoxSampleDownsampleFragmentShader(),
                                   [this, use_binding_layout](GL::Program& prog) {
                                     if (!IsGLES() && !use_binding_layout)
                                       prog.BindFragData(0, "o_col0");
                                   });
    if (!prog)
      return false;

    if (!use_binding_layout)
    {
      prog->Bind();
      prog->Uniform1i("samp0", 0);
    }

    m_downsample_program = std::move(*prog);
  }

  progress.Increment();
#undef UPDATE_PROGRESS

  return true;
}

void GPU_HW_OpenGL::DrawBatchVertices(BatchRenderMode render_mode, u32 base_vertex, u32 num_vertices)
{
  const GL::Program& prog = m_render_programs[static_cast<u8>(render_mode)][static_cast<u8>(m_batch.texture_mode)]
                                             [BoolToUInt8(m_batch.dithering)][BoolToUInt8(m_batch.interlacing)];
  prog.Bind();

  if (m_current_transparency_mode != m_batch.transparency_mode || m_current_render_mode != render_mode)
  {
    m_current_transparency_mode = m_batch.transparency_mode;
    m_current_render_mode = render_mode;
    SetBlendMode();
  }

  SetDepthFunc();

  glDrawArrays(GL_TRIANGLES, m_batch_base_vertex, num_vertices);
}

void GPU_HW_OpenGL::SetBlendMode()
{
  if (UseAlphaBlending(m_current_transparency_mode, m_current_render_mode))
  {
    glEnable(GL_BLEND);
    glBlendEquationSeparate(m_current_transparency_mode == GPUTransparencyMode::BackgroundMinusForeground ?
                              GL_FUNC_REVERSE_SUBTRACT :
                              GL_FUNC_ADD,
                            GL_FUNC_ADD);
    if (m_supports_dual_source_blend)
    {
      glBlendFuncSeparate(GL_ONE, m_supports_dual_source_blend ? GL_SRC1_ALPHA : GL_SRC_ALPHA, GL_ONE, GL_ZERO);
    }
    else
    {
      const float factor =
        (m_current_transparency_mode == GPUTransparencyMode::HalfBackgroundPlusHalfForeground) ? 0.5f : 1.0f;
      glBlendFuncSeparate(GL_ONE, GL_CONSTANT_ALPHA, GL_ONE, GL_ZERO);
      glBlendColor(0.0f, 0.0f, 0.0f, factor);
    }
  }
  else
  {
    glDisable(GL_BLEND);
  }
}

bool GPU_HW_OpenGL::BlitVRAMReplacementTexture(const TextureReplacementTexture* tex, u32 dst_x, u32 dst_y, u32 width,
                                               u32 height)
{
  if (!m_vram_write_replacement_texture.IsValid())
  {
    if (!m_vram_write_replacement_texture.Create(tex->GetWidth(), tex->GetHeight(), 1, GL_RGBA, GL_RGBA,
                                                 GL_UNSIGNED_BYTE, tex->GetPixels(), true) ||
        !m_vram_write_replacement_texture.CreateFramebuffer())
    {
      m_vram_write_replacement_texture.Destroy();
      return false;
    }
  }
  else
  {
    m_vram_write_replacement_texture.Replace(tex->GetWidth(), tex->GetHeight(), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE,
                                             tex->GetPixels());
  }

  glDisable(GL_SCISSOR_TEST);
  m_vram_write_replacement_texture.BindFramebuffer(GL_READ_FRAMEBUFFER);

  dst_y = m_vram_texture.GetHeight() - dst_y - height;
  glBlitFramebuffer(0, tex->GetHeight(), tex->GetWidth(), 0, dst_x, dst_y, dst_x + width, dst_y + height,
                    GL_COLOR_BUFFER_BIT, GL_LINEAR);

  m_vram_read_texture.Bind();
  glEnable(GL_SCISSOR_TEST);
  return true;
}

void GPU_HW_OpenGL::SetDepthFunc()
{
  SetDepthFunc(m_batch.use_depth_buffer ? GL_LEQUAL : (m_batch.check_mask_before_draw ? GL_GEQUAL : GL_ALWAYS));
}

void GPU_HW_OpenGL::SetDepthFunc(GLenum func)
{
  if (m_current_depth_test == func)
    return;

  glDepthFunc(func);
  m_current_depth_test = func;
}

void GPU_HW_OpenGL::SetScissorFromDrawingArea()
{
  int left, top, right, bottom;
  CalcScissorRect(&left, &top, &right, &bottom);

  const int width = right - left;
  const int height = bottom - top;
  const int x = left;
  const int y = m_vram_texture.GetHeight() - bottom;

  Log_DebugPrintf("SetScissor: (%d-%d, %d-%d)", x, x + width, y, y + height);
  glScissor(x, y, width, height);
}

void GPU_HW_OpenGL::UploadUniformBuffer(const void* data, u32 data_size)
{
  const GL::StreamBuffer::MappingResult res = m_uniform_stream_buffer->Map(m_uniform_buffer_alignment, data_size);
  std::memcpy(res.pointer, data, data_size);
  m_uniform_stream_buffer->Unmap(data_size);

  glBindBufferRange(GL_UNIFORM_BUFFER, 1, m_uniform_stream_buffer->GetGLBufferId(), res.buffer_offset, data_size);

  m_renderer_stats.num_uniform_buffer_updates++;
}

void GPU_HW_OpenGL::ClearDisplay()
{
  GPU_HW::ClearDisplay();

  m_host_display->ClearDisplayTexture();

  m_display_texture.BindFramebuffer(GL_DRAW_FRAMEBUFFER);
  glDisable(GL_SCISSOR_TEST);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glEnable(GL_SCISSOR_TEST);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
}

void GPU_HW_OpenGL::UpdateDisplay()
{
  GPU_HW::UpdateDisplay();

  if (g_settings.debugging.show_vram)
  {
    if (IsUsingMultisampling())
    {
      UpdateVRAMReadTexture();

      m_host_display->SetDisplayTexture(reinterpret_cast<void*>(static_cast<uintptr_t>(m_vram_read_texture.GetGLId())),
                                        HostDisplayPixelFormat::RGBA8, m_vram_read_texture.GetWidth(),
                                        static_cast<s32>(m_vram_read_texture.GetHeight()), 0,
                                        m_vram_read_texture.GetHeight(), m_vram_read_texture.GetWidth(),
                                        -static_cast<s32>(m_vram_read_texture.GetHeight()));
    }
    else
    {
      m_host_display->SetDisplayTexture(reinterpret_cast<void*>(static_cast<uintptr_t>(m_vram_texture.GetGLId())),
                                        HostDisplayPixelFormat::RGBA8, m_vram_texture.GetWidth(),
                                        static_cast<s32>(m_vram_texture.GetHeight()), 0, m_vram_texture.GetHeight(),
                                        m_vram_texture.GetWidth(), -static_cast<s32>(m_vram_texture.GetHeight()));
    }
    m_host_display->SetDisplayParameters(VRAM_WIDTH, VRAM_HEIGHT, 0, 0, VRAM_WIDTH, VRAM_HEIGHT,
                                         static_cast<float>(VRAM_WIDTH) / static_cast<float>(VRAM_HEIGHT));
  }
  else
  {
    m_host_display->SetDisplayParameters(m_crtc_state.display_width, m_crtc_state.display_height,
                                         m_crtc_state.display_origin_left, m_crtc_state.display_origin_top,
                                         m_crtc_state.display_vram_width, m_crtc_state.display_vram_height,
                                         GetDisplayAspectRatio());

    const u32 resolution_scale = m_GPUSTAT.display_area_color_depth_24 ? 1 : m_resolution_scale;
    const u32 vram_offset_x = m_crtc_state.display_vram_left;
    const u32 vram_offset_y = m_crtc_state.display_vram_top;
    const u32 scaled_vram_offset_x = vram_offset_x * resolution_scale;
    const u32 scaled_vram_offset_y = vram_offset_y * resolution_scale;
    const u32 display_width = m_crtc_state.display_vram_width;
    const u32 display_height = m_crtc_state.display_vram_height;
    const u32 scaled_display_width = display_width * resolution_scale;
    const u32 scaled_display_height = display_height * resolution_scale;
    const InterlacedRenderMode interlaced = GetInterlacedRenderMode();

    if (IsDisplayDisabled())
    {
      m_host_display->ClearDisplayTexture();
    }
    else if (!m_GPUSTAT.display_area_color_depth_24 && interlaced == GPU_HW::InterlacedRenderMode::None &&
             !IsUsingMultisampling() && (scaled_vram_offset_x + scaled_display_width) <= m_vram_texture.GetWidth() &&
             (scaled_vram_offset_y + scaled_display_height) <= m_vram_texture.GetHeight())
    {
      if (IsUsingDownsampling())
      {
        DownsampleFramebuffer(m_vram_texture, scaled_vram_offset_x, scaled_vram_offset_y, scaled_display_width,
                              scaled_display_height);
      }
      else
      {
        m_host_display->SetDisplayTexture(reinterpret_cast<void*>(static_cast<uintptr_t>(m_vram_texture.GetGLId())),
                                          HostDisplayPixelFormat::RGBA8, m_vram_texture.GetWidth(),
                                          m_vram_texture.GetHeight(), scaled_vram_offset_x,
                                          m_vram_texture.GetHeight() - scaled_vram_offset_y, scaled_display_width,
                                          -static_cast<s32>(scaled_display_height));
      }
    }
    else
    {
      glDisable(GL_BLEND);
      glDisable(GL_SCISSOR_TEST);
      glDisable(GL_DEPTH_TEST);

      m_display_programs[BoolToUInt8(m_GPUSTAT.display_area_color_depth_24)][static_cast<u8>(interlaced)].Bind();
      m_display_texture.BindFramebuffer(GL_DRAW_FRAMEBUFFER);
      m_vram_texture.Bind();

      if (interlaced == InterlacedRenderMode::None && (GLAD_GL_VERSION_4_3 || GLAD_GL_ES_VERSION_3_0))
      {
        static constexpr std::array<GLenum, 1> attachments = {GL_COLOR_ATTACHMENT0};
        glInvalidateFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLsizei>(attachments.size()), attachments.data());
      }

      const u8 height_div2 = BoolToUInt8(interlaced == GPU_HW::InterlacedRenderMode::SeparateFields);
      const u32 reinterpret_field_offset = (interlaced != InterlacedRenderMode::None) ? GetInterlacedDisplayField() : 0;
      const u32 scaled_flipped_vram_offset_y = m_vram_texture.GetHeight() - scaled_vram_offset_y -
                                               reinterpret_field_offset - (scaled_display_height >> height_div2);
      const u32 reinterpret_start_x = m_crtc_state.regs.X * resolution_scale;
      const u32 reinterpret_crop_left = (m_crtc_state.display_vram_left - m_crtc_state.regs.X) * resolution_scale;
      const u32 uniforms[4] = {reinterpret_start_x, scaled_flipped_vram_offset_y, reinterpret_crop_left,
                               reinterpret_field_offset};
      UploadUniformBuffer(uniforms, sizeof(uniforms));
      m_batch_ubo_dirty = true;

      Assert(scaled_display_width <= m_display_texture.GetWidth() &&
             scaled_display_height <= m_display_texture.GetHeight());

      glViewport(0, 0, scaled_display_width, scaled_display_height);
      glBindVertexArray(m_attributeless_vao_id);
      glDrawArrays(GL_TRIANGLES, 0, 3);

      if (IsUsingDownsampling())
      {
        DownsampleFramebuffer(m_display_texture, 0, 0, scaled_display_width, scaled_display_height);
      }
      else
      {
        m_host_display->SetDisplayTexture(reinterpret_cast<void*>(static_cast<uintptr_t>(m_display_texture.GetGLId())),
                                          HostDisplayPixelFormat::RGBA8, m_display_texture.GetWidth(),
                                          m_display_texture.GetHeight(), 0, scaled_display_height, scaled_display_width,
                                          -static_cast<s32>(scaled_display_height));
      }

      // restore state
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
      glBindVertexArray(m_vao_id);
      glViewport(0, 0, m_vram_texture.GetWidth(), m_vram_texture.GetHeight());
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_SCISSOR_TEST);
      m_vram_read_texture.Bind();
      SetBlendMode();
      SetDepthFunc();
    }
  }
}

void GPU_HW_OpenGL::ReadVRAM(u32 x, u32 y, u32 width, u32 height)
{
  if (IsUsingSoftwareRendererForReadbacks())
  {
    ReadSoftwareRendererVRAM(x, y, width, height);
    return;
  }

  // Get bounds with wrap-around handled.
  const Common::Rectangle<u32> copy_rect = GetVRAMTransferBounds(x, y, width, height);
  const u32 encoded_width = (copy_rect.GetWidth() + 1) / 2;
  const u32 encoded_height = copy_rect.GetHeight();

  // Encode the 24-bit texture as 16-bit.
  const u32 uniforms[4] = {copy_rect.left, VRAM_HEIGHT - copy_rect.top - copy_rect.GetHeight(), copy_rect.GetWidth(),
                           copy_rect.GetHeight()};
  m_vram_encoding_texture.BindFramebuffer(GL_DRAW_FRAMEBUFFER);
  m_vram_texture.Bind();
  m_vram_read_program.Bind();
  UploadUniformBuffer(uniforms, sizeof(uniforms));
  glDisable(GL_BLEND);
  glDisable(GL_SCISSOR_TEST);
  glViewport(0, 0, encoded_width, encoded_height);
  glBindVertexArray(m_attributeless_vao_id);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // Readback encoded texture.
  m_vram_encoding_texture.BindFramebuffer(GL_READ_FRAMEBUFFER);
  glPixelStorei(GL_PACK_ALIGNMENT, 2);
  glPixelStorei(GL_PACK_ROW_LENGTH, VRAM_WIDTH / 2);
  glReadPixels(0, 0, encoded_width, encoded_height, GL_RGBA, GL_UNSIGNED_BYTE,
               &m_vram_shadow[copy_rect.top * VRAM_WIDTH + copy_rect.left]);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  RestoreGraphicsAPIState();
}

void GPU_HW_OpenGL::FillVRAM(u32 x, u32 y, u32 width, u32 height, u32 color)
{
  if (IsUsingSoftwareRendererForReadbacks())
    FillSoftwareRendererVRAM(x, y, width, height, color);

  GPU_HW::FillVRAM(x, y, width, height, color);

  const Common::Rectangle<u32> bounds(GetVRAMTransferBounds(x, y, width, height));
  glScissor(bounds.left * m_resolution_scale,
            m_vram_texture.GetHeight() - (bounds.top * m_resolution_scale) - (height * m_resolution_scale),
            width * m_resolution_scale, height * m_resolution_scale);

  // fast path when not using interlaced rendering
  const bool wrapped = IsVRAMFillOversized(x, y, width, height);
  const bool interlaced = IsInterlacedRenderingEnabled();
  if (!wrapped && !interlaced)
  {
    const auto [r, g, b, a] =
      RGBA8ToFloat(m_true_color ? color : VRAMRGBA5551ToRGBA8888(VRAMRGBA8888ToRGBA5551(color)));
    glClearColor(r, g, b, a);
    IsGLES() ? glClearDepthf(a) : glClearDepth(a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    SetScissorFromDrawingArea();
  }
  else
  {
    const VRAMFillUBOData uniforms = GetVRAMFillUBOData(x, y, width, height, color);

    m_vram_fill_programs[BoolToUInt8(wrapped)][BoolToUInt8(interlaced)].Bind();
    UploadUniformBuffer(&uniforms, sizeof(uniforms));
    glDisable(GL_BLEND);
    SetDepthFunc(GL_ALWAYS);
    glBindVertexArray(m_attributeless_vao_id);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    RestoreGraphicsAPIState();
  }
}

void GPU_HW_OpenGL::UpdateVRAM(u32 x, u32 y, u32 width, u32 height, const void* data, bool set_mask, bool check_mask)
{
  if (IsUsingSoftwareRendererForReadbacks())
    UpdateSoftwareRendererVRAM(x, y, width, height, data, set_mask, check_mask);

  const Common::Rectangle<u32> bounds = GetVRAMTransferBounds(x, y, width, height);
  GPU_HW::UpdateVRAM(bounds.left, bounds.top, bounds.GetWidth(), bounds.GetHeight(), data, set_mask, check_mask);

  if (!check_mask)
  {
    const TextureReplacementTexture* rtex = g_texture_replacements.GetVRAMWriteReplacement(width, height, data);
    if (rtex && BlitVRAMReplacementTexture(rtex, x * m_resolution_scale, y * m_resolution_scale,
                                           width * m_resolution_scale, height * m_resolution_scale))
    {
      return;
    }
  }

  const u32 num_pixels = width * height;
  if (m_use_texture_buffer_for_vram_writes || m_use_ssbo_for_vram_writes)
  {
    const auto map_result = m_texture_stream_buffer->Map(sizeof(u16), num_pixels * sizeof(u16));
    std::memcpy(map_result.pointer, data, num_pixels * sizeof(u16));
    m_texture_stream_buffer->Unmap(num_pixels * sizeof(u16));
    m_texture_stream_buffer->Unbind();

    glDisable(GL_BLEND);
    SetDepthFunc((check_mask && !m_pgxp_depth_buffer) ? GL_GEQUAL : GL_ALWAYS);

    m_vram_write_program.Bind();
    if (m_use_ssbo_for_vram_writes)
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_texture_stream_buffer->GetGLBufferId());
    else
      glBindTexture(GL_TEXTURE_BUFFER, m_texture_buffer_r16ui_texture);

    const VRAMWriteUBOData uniforms =
      GetVRAMWriteUBOData(x, y, width, height, map_result.index_aligned, set_mask, check_mask);
    UploadUniformBuffer(&uniforms, sizeof(uniforms));

    // the viewport should already be set to the full vram, so just adjust the scissor
    const Common::Rectangle<u32> scaled_bounds = bounds * m_resolution_scale;
    glScissor(scaled_bounds.left, m_vram_texture.GetHeight() - scaled_bounds.top - scaled_bounds.GetHeight(),
              scaled_bounds.GetWidth(), scaled_bounds.GetHeight());

    glBindVertexArray(m_attributeless_vao_id);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    RestoreGraphicsAPIState();
  }
  else
  {
    if ((x + width) > VRAM_WIDTH || (y + height) > VRAM_HEIGHT || check_mask)
    {
      // CPU round trip if oversized for now.
      Log_WarningPrintf("Oversized/masked VRAM update (%u-%u, %u-%u), CPU round trip", x, x + width, y, y + height);
      ReadVRAM(0, 0, VRAM_WIDTH, VRAM_HEIGHT);
      GPU::UpdateVRAM(x, y, width, height, data, set_mask, check_mask);
      UpdateVRAM(0, 0, VRAM_WIDTH, VRAM_HEIGHT, m_vram_shadow.data(), false, false);
      return;
    }

    GPU_HW::UpdateVRAM(x, y, width, height, data, set_mask, check_mask);

    const auto map_result = m_texture_stream_buffer->Map(sizeof(u32), num_pixels * sizeof(u32));

    // reverse copy the rows so it matches opengl's lower-left origin
    const u32 source_stride = width * sizeof(u16);
    const u8* source_ptr = static_cast<const u8*>(data) + (source_stride * (height - 1));
    const u16 mask_or = set_mask ? 0x8000 : 0x0000;
    u32* dest_ptr = static_cast<u32*>(map_result.pointer);
    for (u32 row = 0; row < height; row++)
    {
      const u8* source_row_ptr = source_ptr;

      for (u32 col = 0; col < width; col++)
      {
        u16 src_col;
        std::memcpy(&src_col, source_row_ptr, sizeof(src_col));
        source_row_ptr += sizeof(src_col);
        *(dest_ptr++) = VRAMRGBA5551ToRGBA8888(src_col | mask_or);
      }

      source_ptr -= source_stride;
    }

    m_texture_stream_buffer->Unmap(num_pixels * sizeof(u32));
    m_texture_stream_buffer->Bind();

    // have to write to the 1x texture first
    if (m_resolution_scale > 1)
      m_vram_encoding_texture.Bind();
    else
      m_vram_texture.Bind();

    // lower-left origin flip happens here
    const u32 flipped_y = VRAM_HEIGHT - y - height;

    // update texture data
    glTexSubImage2D(m_vram_texture.GetGLTarget(), 0, x, flipped_y, width, height, GL_RGBA, GL_UNSIGNED_BYTE,
                    reinterpret_cast<void*>(static_cast<uintptr_t>(map_result.buffer_offset)));
    m_texture_stream_buffer->Unbind();

    if (m_resolution_scale > 1)
    {
      // scale to internal resolution
      const u32 scaled_width = width * m_resolution_scale;
      const u32 scaled_height = height * m_resolution_scale;
      const u32 scaled_x = x * m_resolution_scale;
      const u32 scaled_y = y * m_resolution_scale;
      const u32 scaled_flipped_y = m_vram_texture.GetHeight() - scaled_y - scaled_height;
      glDisable(GL_SCISSOR_TEST);
      m_vram_encoding_texture.BindFramebuffer(GL_READ_FRAMEBUFFER);
      glBlitFramebuffer(x, flipped_y, x + width, flipped_y + height, scaled_x, scaled_flipped_y,
                        scaled_x + scaled_width, scaled_flipped_y + scaled_height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
      glEnable(GL_SCISSOR_TEST);
    }
  }
}

void GPU_HW_OpenGL::CopyVRAM(u32 src_x, u32 src_y, u32 dst_x, u32 dst_y, u32 width, u32 height)
{
  if (IsUsingSoftwareRendererForReadbacks())
    CopySoftwareRendererVRAM(src_x, src_y, dst_x, dst_y, width, height);

  const Common::Rectangle<u32> dst_bounds = GetVRAMTransferBounds(dst_x, dst_y, width, height);
  const Common::Rectangle<u32> src_bounds = GetVRAMTransferBounds(src_x, src_y, width, height);
  const bool src_dirty = m_vram_dirty_rect.Intersects(src_bounds);

  if (UseVRAMCopyShader(src_x, src_y, dst_x, dst_y, width, height))
  {
    if (src_dirty)
      UpdateVRAMReadTexture();
    IncludeVRAMDirtyRectangle(dst_bounds);

    VRAMCopyUBOData uniforms = GetVRAMCopyUBOData(src_x, src_y, dst_x, dst_y, width, height);
    uniforms.u_src_y = m_vram_texture.GetHeight() - uniforms.u_src_y - uniforms.u_height;
    uniforms.u_dst_y = m_vram_texture.GetHeight() - uniforms.u_dst_y - uniforms.u_height;
    UploadUniformBuffer(&uniforms, sizeof(uniforms));

    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_BLEND);
    SetDepthFunc((m_GPUSTAT.check_mask_before_draw && !m_pgxp_depth_buffer) ? GL_GEQUAL : GL_ALWAYS);

    const Common::Rectangle<u32> dst_bounds_scaled(dst_bounds * m_resolution_scale);
    glViewport(dst_bounds_scaled.left,
               m_vram_texture.GetHeight() - dst_bounds_scaled.top - dst_bounds_scaled.GetHeight(),
               dst_bounds_scaled.GetWidth(), dst_bounds_scaled.GetHeight());
    m_vram_read_texture.Bind();
    m_vram_copy_program.Bind();
    glBindVertexArray(m_attributeless_vao_id);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    RestoreGraphicsAPIState();

    if (m_GPUSTAT.check_mask_before_draw)
      m_current_depth++;

    return;
  }

  GPU_HW::CopyVRAM(src_x, src_y, dst_x, dst_y, width, height);

  src_x *= m_resolution_scale;
  src_y *= m_resolution_scale;
  dst_x *= m_resolution_scale;
  dst_y *= m_resolution_scale;
  width *= m_resolution_scale;
  height *= m_resolution_scale;

  // lower-left origin flip
  src_y = m_vram_texture.GetHeight() - src_y - height;
  dst_y = m_vram_texture.GetHeight() - dst_y - height;

  if (GLAD_GL_VERSION_4_3)
  {
    glCopyImageSubData(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, src_x, src_y, 0,
                       m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, dst_x, dst_y, 0, width, height, 1);
  }
  else if (GLAD_GL_EXT_copy_image)
  {
    glCopyImageSubDataEXT(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, src_x, src_y, 0,
                          m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, dst_x, dst_y, 0, width, height, 1);
  }
  else if (GLAD_GL_OES_copy_image)
  {
    glCopyImageSubDataOES(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, src_x, src_y, 0,
                          m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, dst_x, dst_y, 0, width, height, 1);
  }
  else
  {
    // glBlitFramebufer with same source/destination should be legal, but on Mali (at least Bifrost) it breaks.
    // So, blit from the shadow texture, like in the other renderers.
    if (src_dirty)
      UpdateVRAMReadTexture();

    glDisable(GL_SCISSOR_TEST);
    m_vram_read_texture.BindFramebuffer(GL_READ_FRAMEBUFFER);
    glBlitFramebuffer(src_x, src_y, src_x + width, src_y + height, dst_x, dst_y, dst_x + width, dst_y + height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glEnable(GL_SCISSOR_TEST);
  }

  IncludeVRAMDirtyRectangle(dst_bounds);
}

void GPU_HW_OpenGL::UpdateVRAMReadTexture()
{
  const auto scaled_rect = m_vram_dirty_rect * m_resolution_scale;
  const u32 width = scaled_rect.GetWidth();
  const u32 height = scaled_rect.GetHeight();
  const u32 x = scaled_rect.left;
  const u32 y = m_vram_texture.GetHeight() - scaled_rect.top - height;
  const bool multisampled = m_vram_texture.IsMultisampled();

  if (!multisampled && GLAD_GL_VERSION_4_3)
  {
    glCopyImageSubData(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0,
                       m_vram_read_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0, width, height, 1);
  }
  else if (!multisampled && GLAD_GL_EXT_copy_image)
  {
    glCopyImageSubDataEXT(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0,
                          m_vram_read_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0, width, height, 1);
  }
  else if (!multisampled && GLAD_GL_OES_copy_image)
  {
    glCopyImageSubDataOES(m_vram_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0,
                          m_vram_read_texture.GetGLId(), m_vram_texture.GetGLTarget(), 0, x, y, 0, width, height, 1);
  }
  else
  {
    m_vram_read_texture.BindFramebuffer(GL_DRAW_FRAMEBUFFER);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_vram_fbo_id);
    glDisable(GL_SCISSOR_TEST);
    glBlitFramebuffer(x, y, x + width, y + height, x, y, x + width, y + height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glEnable(GL_SCISSOR_TEST);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_vram_fbo_id);
  }

  GPU_HW::UpdateVRAMReadTexture();
}

void GPU_HW_OpenGL::UpdateDepthBufferFromMaskBit()
{
  if (m_pgxp_depth_buffer)
    return;

  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  glDepthFunc(GL_ALWAYS);

  m_vram_texture.Bind();
  m_vram_update_depth_program.Bind();
  glBindVertexArray(m_attributeless_vao_id);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  glBindVertexArray(m_vao_id);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_SCISSOR_TEST);

  m_vram_read_texture.Bind();
}

void GPU_HW_OpenGL::ClearDepthBuffer()
{
  glDisable(GL_SCISSOR_TEST);
  IsGLES() ? glClearDepthf(1.0f) : glClearDepth(1.0f);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_SCISSOR_TEST);
  m_last_depth_z = 1.0f;
}

void GPU_HW_OpenGL::DownsampleFramebuffer(GL::Texture& source, u32 left, u32 top, u32 width, u32 height)
{
  DebugAssert(m_downsample_mode != GPUDownsampleMode::Adaptive);
  DownsampleFramebufferBoxFilter(source, left, top, width, height);
}

void GPU_HW_OpenGL::DownsampleFramebufferBoxFilter(GL::Texture& source, u32 left, u32 top, u32 width, u32 height)
{
  const u32 ds_left = left / m_resolution_scale;
  const u32 ds_top = top / m_resolution_scale;
  const u32 ds_width = width / m_resolution_scale;
  const u32 ds_height = height / m_resolution_scale;

  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_SCISSOR_TEST);
  glViewport(ds_left, m_downsample_texture.GetHeight() - ds_top - ds_height, ds_width, ds_height);
  glBindVertexArray(m_attributeless_vao_id);
  source.Bind();
  m_downsample_texture.BindFramebuffer(GL_DRAW_FRAMEBUFFER);
  m_downsample_program.Bind();
  glDrawArrays(GL_TRIANGLES, 0, 3);

  RestoreGraphicsAPIState();

  m_host_display->SetDisplayTexture(reinterpret_cast<void*>(static_cast<uintptr_t>(m_downsample_texture.GetGLId())),
                                    HostDisplayPixelFormat::RGBA8, m_downsample_texture.GetWidth(),
                                    m_downsample_texture.GetHeight(), ds_left,
                                    m_downsample_texture.GetHeight() - ds_top, ds_width, -static_cast<s32>(ds_height));
}

std::unique_ptr<GPU> GPU::CreateHardwareOpenGLRenderer()
{
  return std::make_unique<GPU_HW_OpenGL>();
}
