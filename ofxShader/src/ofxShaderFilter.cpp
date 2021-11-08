#include "ofxShaderFilter.h"
#include <regex>

ofxShaderFilter::ofxShaderFilter() {
//     ofAddListener(m_shader.onChange, this, &ofxShaderFilter::_reload);
    ofAddListener(m_shader.onLoad, this, &ofxShaderFilter::_reload);
}

ofxShaderFilter::~ofxShaderFilter() {
}

void ofxShaderFilter::allocate(int _width, int _height) {
    m_width = _width;
    m_height = _height;

    m_fbo.allocate(m_width, m_height);

    for (unsigned int i = 0; i < m_buffers.size(); i++) {
        m_buffers[i].allocate(m_width, m_height);
    }
}

bool ofxShaderFilter::load(const string &_frag) {
    // Clear buffers
    m_buffers.clear();
    m_buffers_shaders.clear();

    // Load the main shader
   bool rta = m_shader.load("none.vert", _frag, "none.geom");
    
    _updateBuffers();
    return rta;
}

void ofxShaderFilter::_reload(bool &_args) {
    _updateBuffers();
}

//--------------------------------------------------------------
void ofxShaderFilter::setUniform2f(const string &_name, const glm::vec2 & v) {
	setUniform2f(_name, v.x, v.y);
}

//--------------------------------------------------------------
void ofxShaderFilter::setUniform3f(const string &_name, const glm::vec3 & v) {
	setUniform3f(_name, v.x, v.y, v.z);
}

//--------------------------------------------------------------
void ofxShaderFilter::setUniform4f(const string &_name, const glm::vec4 & v) {
	setUniform4f(_name, v.x, v.y, v.z, v.w);
}

//--------------------------------------------------------------
void ofxShaderFilter::setUniform4f(const string &_name, const ofFloatColor & v) {
	setUniform4f(_name, v.r, v.g, v.b, v.a);
}

// set a single uniform value
void ofxShaderFilter::setUniform1i(const string &_name, int v1) {
    m_uniformsData[_name].value[0] = float(v1);
    m_uniformsData[_name].size = 1;
    m_uniformsData[_name].bInt = true;
}

void ofxShaderFilter::setUniform2i(const string &_name, int v1, int v2) {
    m_uniformsData[_name].value[0] = float(v1);
    m_uniformsData[_name].value[1] = float(v2);
    m_uniformsData[_name].size = 2;
    m_uniformsData[_name].bInt = true;
}

void ofxShaderFilter::setUniform3i(const string &_name, int v1, int v2, int v3) {
    m_uniformsData[_name].value[0] = float(v1);
    m_uniformsData[_name].value[1] = float(v2);
    m_uniformsData[_name].value[2] = float(v3);
    m_uniformsData[_name].size = 3;
    m_uniformsData[_name].bInt = true;
}

void ofxShaderFilter::setUniform4i(const string &_name, int v1, int v2, int v3, int v4) {
    m_uniformsData[_name].value[0] = float(v1);
    m_uniformsData[_name].value[1] = float(v2);
    m_uniformsData[_name].value[2] = float(v3);
    m_uniformsData[_name].value[3] = float(v4);
    m_uniformsData[_name].size = 4;
    m_uniformsData[_name].bInt = true;
}

void ofxShaderFilter::setUniform1f(const string &_name, float v1) {
    m_uniformsData[_name].value[0] = v1;
    m_uniformsData[_name].size = 1;
    m_uniformsData[_name].bInt = false;
}

void ofxShaderFilter::setUniform2f(const string &_name, float v1, float v2) {
    m_uniformsData[_name].value[0] = v1;
    m_uniformsData[_name].value[1] = v2;
    m_uniformsData[_name].size = 2;
    m_uniformsData[_name].bInt = false;
}

void ofxShaderFilter::setUniform3f(const string &_name, float v1, float v2, float v3) {
    m_uniformsData[_name].value[0] = v1;
    m_uniformsData[_name].value[1] = v2;
    m_uniformsData[_name].value[2] = v3;
    m_uniformsData[_name].size = 3;
    m_uniformsData[_name].bInt = false;
}

void ofxShaderFilter::setUniform4f(const string &_name, float v1, float v2, float v3, float v4) {
    m_uniformsData[_name].value[0] = v1;
    m_uniformsData[_name].value[1] = v2;
    m_uniformsData[_name].value[2] = v3;
    m_uniformsData[_name].value[3] = v4;
    m_uniformsData[_name].size = 4;
    m_uniformsData[_name].bInt = false;
}

void ofxShaderFilter::setUniformTexture(const string &_name, ofBaseHasTexture& img) {
    m_textures[_name] = &img.getTexture();
}

void ofxShaderFilter::setUniformTexture(const string &_name, ofTexture& img) {
    m_textures[_name] = &img;
}

int ofxShaderFilter::getTotalBuffers() {
    std::string source = m_shader.getShaderSource(GL_FRAGMENT_SHADER);

    std::vector<std::string> lines = ofSplitString(source, "\n");
    std::vector<std::string> results;

    std::regex re(R"((?:^\s*#if|^\s*#elif)(?:\s+)(defined\s*\(\s*BUFFER_)(\d+)(?:\s*\))|(?:^\s*#ifdef\s+BUFFER_)(\d+))");
    std::smatch match;

    for (unsigned int l = 0; l < lines.size(); l++) {
        if (std::regex_search(lines[l], match, re)) {

            // for (int i = 0; i < match.size(); i++) {
            //     cout << i << " -> " << std::ssub_match(match[i]).str() << endl;
            // }

            string number = std::ssub_match(match[2]).str();
            if (number.size() == 0) {
                number = std::ssub_match(match[3]).str();
            }

            bool already = false;
            for (unsigned int i = 0; i < results.size(); i++) {
                if (results[i] == number) {
                    already = true;
                    break;
                }
            }

            if (!already) {
                results.push_back(number);
            }
        }
    }

    return results.size();
}

void ofxShaderFilter::_updateBuffers() {
    int total = getTotalBuffers();

//     cout << "TOTAL BUFFERS: " << total << endl; 
    
    if ( total != int(m_buffers.size()) ) {
        m_buffers.clear();
        m_buffers_shaders.clear();

        for (int i = 0; i < total; i++) {
            // New FBO
            ofFbo new_fbo;
            new_fbo.allocate(m_width, m_height);
            m_buffers.push_back( new_fbo );

            // New SHADER
            ofxShader new_shader = ofxShader();
            new_shader.load("none.vert", m_shader.getFilename(GL_FRAGMENT_SHADER), "none.geom");
            new_shader.addDefineKeyword("BUFFER_" + ofToString(i) + " u_buffer" + ofToString(i) );
            m_buffers_shaders.push_back( new_shader );
        }

    }
    else {
        for (unsigned int i = 0; i < m_buffers_shaders.size(); i++) {
            m_buffers_shaders[i].load("none.vert", m_shader.getFilename(GL_FRAGMENT_SHADER), "none.geom");
        }
    }
}

void setUniforms(ofShader &_shader, UniformDataList &_uniforms ) {
    for (UniformDataList::iterator it = _uniforms.begin(); it != _uniforms.end(); ++it) {
        // TODO:
        //      - only update on change
        //
        if (it->second.bInt) {
            switch (it->second.size) {
                case 1:
                    _shader.setUniform1i(it->first, int(it->second.value[0]));
                    break;
                case 2:
                    _shader.setUniform2i(it->first, int(it->second.value[0]), int(it->second.value[1]));
                    break;
                case 3:
                    _shader.setUniform3i(it->first, int(it->second.value[0]), int(it->second.value[1]), int(it->second.value[2]));
                    break;
                case 4:
                    _shader.setUniform4i(it->first, int(it->second.value[0]), int(it->second.value[1]), int(it->second.value[2]), int(it->second.value[3]));
                    break;
            }
        }
        else {
            switch (it->second.size) {
                case 1:
                    _shader.setUniform1f(it->first, it->second.value[0]);
                    break;
                case 2:
                    _shader.setUniform2f(it->first, it->second.value[0], it->second.value[1]);
                    break;
                case 3:
                    _shader.setUniform3f(it->first, it->second.value[0], it->second.value[1], it->second.value[2]);
                    break;
                case 4:
                    _shader.setUniform4f(it->first, it->second.value[0], it->second.value[1], it->second.value[2], it->second.value[3]);
                    break;
            }
        }
    }
}

void ofxShaderFilter::render() {

    // Update buffers
    for (unsigned int i = 0; i < m_buffers.size(); i++) {
        m_buffers[i].begin();
        m_buffers_shaders[i].begin();
        
        // Pass textures for the other buffers
        int textureIndex = 1;
        for (unsigned int j = 0; j < m_buffers.size(); j++) {
            if (i != j) {
                m_buffers_shaders[i].setUniformTexture("u_buffer" + ofToString(j), m_buffers[j], textureIndex );
                textureIndex++;
            }
        }

        for (TextureList::iterator it = m_textures.begin(); it != m_textures.end(); ++it) {
            m_buffers_shaders[i].setUniformTexture(it->first, *it->second, textureIndex );
            textureIndex++;
        }

        // Pass all uniforms
        setUniforms(m_buffers_shaders[i], m_uniformsData);
        m_buffers_shaders[i].setUniform2f("u_resolution", getWidth(), getHeight());

        m_buffers[i].draw(0, 0);
        m_buffers_shaders[i].end();
        m_buffers[i].end();
    }

    // Main shader
    m_fbo.begin();
    m_shader.begin();

    int textureIndex = 1;
    for (unsigned int i = 0; i < m_buffers.size(); i++) {
        m_shader.setUniformTexture("u_buffer" + ofToString(i), m_buffers[i], textureIndex );
        textureIndex++;
    }

    for (TextureList::iterator it = m_textures.begin(); it != m_textures.end(); ++it) {
        m_shader.setUniformTexture(it->first, *it->second, textureIndex );
        textureIndex++;
    }

    // Pass all uniforms
    setUniforms(m_shader, m_uniformsData);

    m_shader.setUniform2f("u_resolution", getWidth(), getHeight());

    m_fbo.draw(0, 0);

    m_shader.end();
    m_fbo.end();
}

void ofxShaderFilter::draw(float _x, float _y, float _width, float _height) const {
    m_fbo.draw(_x, _y, _width, _height);
}

// ofBaseHasTexture
ofTexture& ofxShaderFilter::getTexture() {
    return m_fbo.getTexture();
}

const ofTexture& ofxShaderFilter::getTexture() const {
    return m_fbo.getTexture();
}
