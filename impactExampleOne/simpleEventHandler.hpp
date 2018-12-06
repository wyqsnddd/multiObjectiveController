//==============================================================================
class simpleEventHandler : public osgGA::GUIEventHandler
{
	public:

		simpleEventHandler(/*Pass in any necessary arguments*/)
		{
			// Set up the customized event handler
		}

		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter&) override
		{
			if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
			{ 
				if(ea.getKey() == 'q')
				{ 
					std::cout << "Lowercase q pressed" << std::endl;
					return true;
				}
				else if(ea.getKey() == 'Q')
				{ 
					std::cout << "Capital Q pressed" << std::endl;
					return true;
				}
				else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
				{ 
					std::cout << "Left arrow key pressed" << std::endl;
					return true;
				}
				else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
				{ 
					std::cout << "Right arrow key pressed" << std::endl;
					return true;
				}
			}
			else if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
			{ 
				if(ea.getKey() == 'q')
				{ 
					std::cout << "Lowercase q released" << std::endl;
					return true;
				}
				else if(ea.getKey() == 'Q')
				{ 
					std::cout << "Capital Q released" << std::endl;
					return true;
				}
				else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
				{ 
					std::cout << "Left arrow key released" << std::endl;
					return true;
				}
				else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
				{ 
					std::cout << "Right arrow key released" << std::endl;
					return true;
				}
			}

			// The return value should be 'true' if the input has been fully handled
			// and should not be visible to any remaining event handlers. It should be
			// false if the input has not been fully handled and should be viewed by
			// any remaining event handlers.
			return false;
		} 

};


