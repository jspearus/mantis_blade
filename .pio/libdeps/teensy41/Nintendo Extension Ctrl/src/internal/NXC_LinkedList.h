/*
*  Project     Nintendo Extension Controller Library
*  @author     David Madison
*  @link       github.com/dmadison/NintendoExtensionCtrl
*  @license    LGPLv3 - Copyright (c) 2021 David Madison
*
*  This file is part of the Nintendo Extension Controller Library.
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NXC_LINKED_LIST_H
#define NXC_LINKED_LIST_H


namespace NintendoExtensionCtrl {
	class ExtensionController;

	class ExtensionList {
	public:
		class Node {
		public:
			friend class ExtensionList;  // for changing next ptr

			Node(ExtensionList&, ExtensionController&);
			~Node();

			ExtensionController& getController() const;
			Node* getNext() const;

		private:
			ExtensionController& controller;

			ExtensionList& list;
			Node* next = nullptr;
		};
		friend Node::Node(ExtensionList&, ExtensionController&), Node::~Node();  // for add/remove access

		Node* getHead() const;

	protected:
		void add(Node* node);
		void remove(Node* node);

	private:
		Node* head = nullptr;
	};
}

#endif
