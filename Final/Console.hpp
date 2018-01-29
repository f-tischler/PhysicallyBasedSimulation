#pragma once

#include <SFML/Graphics.hpp>

#include <iomanip>
#include <mutex>
#include <sstream>
#include <unordered_map>

class Console
{
private:
	Console()
	{

	}

public:
	static Console& instance()
	{
		static Console console;
		return console;
	}

	void init()
	{
		if (!font.loadFromFile("arial.ttf"))
		{
			// error...
		}

		m_text.setFont(font);
		m_text.setCharacterSize(12);
		m_text.setFillColor(sf::Color::White);
	}

	template<typename T>
	void set_param(const std::string& name, const T& value)
	{
		std::stringstream stm;
		stm << std::fixed << std::setprecision(2) << value;

		m_params[name] = stm.str();
		update();
	}

	void print(sf::RenderWindow& window)
	{
		std::lock_guard <decltype(m_lock)> ga{ m_lock };

		window.draw(m_text);
	}

private:
	void update()
	{
		std::stringstream stm;

		for (auto itr : m_params)
			stm << itr.first + ": " + itr.second << std::endl;

		{
			std::lock_guard <decltype(m_lock)> ga{ m_lock };
			m_text.setString(stm.str());
		}
	}

	std::mutex m_lock;
	sf::Text m_text;
	std::unordered_map<std::string, std::string> m_params;
	sf::Font font;
};